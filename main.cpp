/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 *
 */

#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <fcntl.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "carutils.h"

#define DBC_NAMELEN_MAX 64
#define NODE_TAG "canbus"

#define rlog rclcpp::get_logger(NODE_TAG)
#define ii(...) RCLCPP_INFO(rlog, __VA_ARGS__);
#define ww(...) RCLCPP_WARN(rlog, __VA_ARGS__);
#define ee(...) RCLCPP_ERROR(rlog, __VA_ARGS__);

namespace {

int can_open(const char *ifname)
{
	struct sockaddr_can addr;
	struct ifreq ifr;
	int sd;
	int flags;
	long unsigned int mtu;

	if ((sd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ee("Failed to create socket: %s", strerror(errno));
		return -1;
	}

	strncpy(ifr.ifr_name, ifname, sizeof(ifr.ifr_name));
	if (ioctl(sd, SIOCGIFMTU, &ifr) < 0) {
		ee("SIOCGIFMTU failed, sd=%d: %s", sd, strerror(errno));
		goto err;
	}

	mtu = ifr.ifr_mtu;
	if (mtu > CAN_MTU) {
		ww("%s: ignore CAN FD capability, mtu %lu ? %lu\n", ifname, mtu,
		 CAN_MTU);
	}

	if (ioctl(sd, SIOCGIFINDEX, &ifr) < 0) {
		ee("SIOCGIFINDEX failed, sd=%d: %s", sd, strerror(errno));
		goto err;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ee("Failed to bind socket sd=%d: %s", sd, strerror(errno));
		goto err;
	}

	flags = fcntl(sd, F_GETFL, 0);
	fcntl(sd, F_SETFL, flags | O_NONBLOCK);
	return sd;
err:
	close(sd);
	return -1;
}

void normalize_string(char *str)
{
	for (int16_t i = strlen(str); i >= 0; --i) {
		if (!isalnum(str[i]) && !isblank(str[i]) && str[i] != '_')
			str[i] = '\0';
	}
}

const struct can_object *get_can_object(char *str, uint8_t len)
{
	uint16_t i = 0;
	uint16_t idx;

	for (i = 0; i < len; ++i) {
		if (isalpha(str[i]))
			break;
	}

	if (i == len - 1)
		return nullptr;

	idx = i;
	for (i = 0; i < ARRAY_SIZE(can_objects_); ++i) {
		if (strcmp(&str[idx], can_objects_[i].name) == 0)
			return &can_objects_[i];
	}

	return nullptr;
}

const struct can_signal *get_can_signal(const struct can_object *obj,
 char *str, uint8_t len)
{
	uint16_t i = 0;
	uint16_t idx;

	for (i = 0; i < len; ++i) {
		if (isalpha(str[i]))
			break;
	}

	idx = i;
	if (i == len - 1)
		return nullptr;

	for (i = 0; i < obj->sigcnt; ++i) {
		if (strcmp(&str[idx], obj->signals[i].name) == 0)
			return &obj->signals[i];
	}

	return nullptr;
}

} /* namespace */

class CanBridge: public rclcpp::Node
{
public:
	CanBridge();
	~CanBridge();

private:
	struct topic {
#ifdef PUBLISH_STR
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str;
#endif
		rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr bin;
		const struct can_object *obj;
		const struct can_signal *sig;
	};

	void readLoop(int sd, struct can_frame *);
	void loop(const char *name);
	void config(const char *path);
	void init(const char *dev, const char *cfg);
	int read(int sd, struct can_frame *);
	void publish(struct can_frame *);
	bool parseSignal(char *str, const struct can_object *,
	 const struct can_signal *, uint32_t line);
	bool parseBusObject(char *str, const struct can_object **,
	 uint32_t line);

	std::vector<std::thread> jobs_;
	std::vector<struct topic> topics_;
	bool done_ = false;
};

CanBridge::~CanBridge()
{
	done_ = true;
	for (auto &job : jobs_) {
		if (job.joinable())
			job.join();
	}

	ii("Stopped");
}

CanBridge::CanBridge(): Node(NODE_TAG)
{
	const char *dev = getenv("CAN_DEVICE");
	if (!dev) {
		ee("CAN_DEVICE variable is not set");
		exit(1);
	}

	const char *cfg = getenv("CAN_CONFIG");
	if (!cfg) {
		ee("CAN_CONFIG variable is not set");
		exit(1);
	}

	init(dev, cfg);
}

#define create_str_publisher(name) \
	this->create_publisher<std_msgs::msg::String>(name, 10)
#define create_bin_publisher(name) \
	this->create_publisher<geometry_msgs::msg::Point32>(name, 10)

bool CanBridge::parseSignal(char *str, const struct can_object *obj,
 const struct can_signal *sig, uint32_t line)
{
	if (!isblank(str[2])) {
		ee("Line %u: blank is expected after 'SG'", line);
		return false;
	} else if (!(sig = get_can_signal(obj, &str[3], sizeof(str) - 3))) {
		ee("Line %u: bad signal name %s", line, &str[3]);
		return false;
	} else {
		CanBridge::topic topic;
		std::string name = "/" NODE_TAG "/";
		name += std::string(obj->name) + "/";
		name += std::string(sig->name);

		if (!(topic.bin = create_bin_publisher(name))) {
			ee("Failed to create bin topic %s, line %u", name, line);
			return false;
		}

#ifdef PUBLISH_STR
		name += "_str";
		if (!(topic.str = create_str_publisher(name))) {
			ee("Failed to create str topic %s, line %u", name, line);
			return false;
		}
#endif

		topic.obj = obj;
		topic.sig = sig;
		topics_.push_back(std::move(topic));
		ii("Created topic %s", name.c_str());
		return true;
	}
}

bool CanBridge::parseBusObject(char *str, const struct can_object **obj,
 uint32_t line)
{
	if (!isblank(str[2])) {
		ee("Line %u: blank is expected after 'BO'", line);
		return false;
	} else if (!(*obj = get_can_object(&str[3], sizeof(str) - 3))) {
		ee("Line %u: bad object name %s", line, &str[3]);
		return false;
	}

	return true;
}

void CanBridge::config(const char *path)
{
	FILE *f = fopen(path, "ro");
	if (!f) {
		ee("Unable to open file %s: %s", path, strerror(errno));
		exit(1);
	}

	ii("Parsing signals ...");
	char str[DBC_NAMELEN_MAX];
	uint32_t line = 1;
	const struct can_object *obj = nullptr;
	const struct can_signal *sig = nullptr;

	while (fgets(str, sizeof(str), f)) {
		normalize_string(str);

		if (str[0] == 'B' && str[1] == 'O') {
			if (!parseBusObject(str, &obj, line))
				break;
		} else if (obj && str[0] == 'S' && str[1] == 'G') {
			if (!parseSignal(str, obj, sig, line))
				break;
		}

		line++;
	}

	fclose(f);
}

void CanBridge::init(const char *dev, const char *cfg)
{
	struct stat st;

	if (stat(cfg, &st) < 0) {
		ee("%s: path error, %s", cfg, strerror(errno));
		exit(1);
	}

	if ((st.st_mode & S_IFMT) != S_IFREG) {
		ee("%s: is not a regular file", cfg);
		exit(1);
	}

	ii("Device %s config %s", dev, cfg);
	config(cfg);

	if (!topics_.size()) {
		ee("Failed to create topics for dev %s", dev);
		exit(1);
	}

	loop(dev);
}

int CanBridge::read(int sd, struct can_frame *frame)
{
	int ret = 0;

	while (!done_) {
		errno = 0;
		ret += ::read(sd, frame, sizeof(*frame));

		if (errno == EINTR && ret != CAN_MTU)
			continue;
		else if (ret < 0 || errno)
			break;
		else
			return ret;
	}

	if (errno != EAGAIN) {
		ee("Failed to read %u bytes from sd %d: %s", CAN_MTU, sd,
		 strerror(errno));
	}
	return -1;
}

void CanBridge::publish(struct can_frame *frame)
{
	uint64_t data = *((uint64_t *) frame->data);
	struct timespec now;

        clock_gettime(CLOCK_MONOTONIC, &now);

	for (auto &topic: topics_) {
		if (frame->can_id != topic.obj->id)
			continue;

		float val = topic.sig->decode(data);
#ifdef PUBLISH_STR
		auto str_msg = std_msgs::msg::String();
		str_msg.data = std::to_string(now.tv_sec) + ".";
		str_msg.data += std::to_string(now.tv_nsec) + " ";
		str_msg.data += std::to_string(val) + " " + topic.sig->units;
		topic.str->publish(str_msg);
#endif
		auto bin_msg = geometry_msgs::msg::Point32();
		bin_msg.x = now.tv_sec;
		bin_msg.y = now.tv_nsec;
		bin_msg.z = val;
		topic.bin->publish(bin_msg);
	}
}

void CanBridge::readLoop(int sd, struct can_frame *frame)
{
	while (!done_) {
		int rc = read(sd, frame);

		if (errno == EAGAIN) {
			usleep(10000);
		} else if (rc < 0 && errno == ENXIO) {
			close(sd);
			break; /* re-create socket */
		} else if (rc < 0 || errno) {
			sleep(1); /* relax and re-read */
			continue;
		} else if (rc == 0) {
			sleep(1); /* relax and re-read */
			continue;
		} else if (topics_.size()) {
			publish(frame);
		}
	}
}

void CanBridge::loop(const char *name)
{
	auto t = std::thread { [=]() -> void {
		int sd = -1;
		while (!done_) {
			struct can_frame frame;

			ii("Open bus %s", name);
			if ((sd = can_open(name)) < 0) {
				sleep(1); /* relax and retry */
				continue;
			}

			readLoop(sd, &frame);
		}
		close(sd);
		ii("Close device %s", name);
	}};

	ii("Run job %s", name);
	jobs_.push_back(move(t));
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CanBridge>());
	rclcpp::shutdown();
	return 0;
}
