#pragma once

#include <elka_comm/common/elka.h>
#include <elka_comm/common/elka_comm.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <string>
#include <elka_defines.h>

class SerialHandler {
public:
  SerialHandler();

  ~SerialHandler();

  bool connect(const std::string &port_name, int baud);

  void start_receive();

  void send(uint8_t *tx_buffer, uint8_t len);

	int write_elka_msg(elka_msg_s &elka_msg);
	int write_elka_msg(elka_msg_ack_s &elka_msg);

  void data_cb(const boost::system::error_code & e,
               std::size_t size);

  inline bool quit() {return _quit_flag;}


private:
  boost::asio::io_service _io;
  boost::asio::serial_port _ser_port;
  boost::thread _t;
	uint8_t _buf[MAX_ELKA_MSG_LEN];

  bool _quit_flag;

};
