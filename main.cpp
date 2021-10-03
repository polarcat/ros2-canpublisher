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
#include "carutils.h"

#define MAX_INTERFACES 4
#define DBC_NAMELEN_MAX 64
#define NODE_TAG "canbus"
#define CONFIG_DIR "/etc/ros2/can"

#define rlog rclcpp::get_logger("canbus")
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
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr val;
		const struct can_object *obj;
		const struct can_signal *sig;
	};

	void readLoop(int sd, struct can_frame *);
	void loop(std::string name);
	void filter(const char *name);
	void init();
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
	init();
}

#define create_publisher(name) \
	this->create_publisher<std_msgs::msg::String>(name, 10)

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
		std::string name = "/canbus/";
		name += std::string(obj->name) + "/";
		name += std::string(sig->name);

		if (!(topic.val = create_publisher(name))) {
			ee("Failed to create topic %s, line %u", name, line);
			return false;
		}

		topic.obj = obj;
		topic.sig = sig;
		topics_.push_back(std::move(topic));
		ii("Created topic %s", name.c_str());
		return true;
	}
}

#undef create_publisher

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

void CanBridge::filter(const char *name)
{
	std::string path = CONFIG_DIR + std::string("/") + name + "/signals";
	FILE *f = fopen(path.c_str(), "ro");
	if (!f) {
		ee("Unable to open file %s: %s", path.c_str(), strerror(errno));
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

void CanBridge::init()
{
	DIR *dir = opendir(CONFIG_DIR);
	if (!dir) {
		ee("Failed to open dir %s: %s", CONFIG_DIR, strerror(errno));
		exit(1);
	}

	ii("Preparing interfaces ...");
	uint8_t count = 0;
	bool found = false;
	struct dirent *ent;

	while ((ent = readdir(dir))) {
		if (ent->d_name[0] == '.' || ent->d_name[1] == '.') {
			continue;
		} else if (++count >= MAX_INTERFACES) { /* sanity check */
			ww("Number of interfaces exceeds %u",
			 MAX_INTERFACES);
			break;
		}

		filter(ent->d_name);

		if (!topics_.size()) {
			ee("Failed to create topics for bus %s", ent->d_name);
			exit(1);
		}

		loop(std::string(basename(ent->d_name)));
		found = true;
	}

	if (!found) {
		ee("No configured interfaces");
		exit(1);
	}
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

	for (auto &topic: topics_) {
		if (frame->can_id != topic.obj->id)
			continue;

		float val = topic.sig->decode(data);
		auto msg = std_msgs::msg::String();
		msg.data = std::to_string(val) + " " + topic.sig->units;
		topic.val->publish(msg);
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

void CanBridge::loop(std::string name)
{
	auto t = std::thread { [=]() -> void {
		int sd = -1;
		while (!done_) {
			struct can_frame frame;

			ii("Open bus %s", name.c_str());
			if ((sd = can_open(name.c_str())) < 0) {
				sleep(1); /* relax and retry */
				continue;
			}

			readLoop(sd, &frame);
		}
		close(sd);
		ii("Close bus %s", name.c_str());
	}};

	ii("Run job %s", name.c_str());
	jobs_.push_back(move(t));
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CanBridge>());
	rclcpp::shutdown();
	return 0;
}
