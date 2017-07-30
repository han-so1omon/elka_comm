#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <elka_log.h>
#include <elka_defines.h>
#include <elka_comm/common/boost_uart.h>

SerialHandler::SerialHandler() : _ser_port(_io), _quit_flag(false){};

SerialHandler::~SerialHandler() {
  _io.stop();
  _t.join();
}

bool SerialHandler::connect(
	const std::string& port_name,
	int baud = 115200) {
	using namespace boost::asio;
	boost::system::error_code ec;
	_ser_port.open(port_name, ec);

	if (ec) {
		LOG_WARN("serial port %s failed to open", port_name.c_str());
		return false;
	}

	//Setup port
	_ser_port.set_option(serial_port::baud_rate(baud));
	_ser_port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));

	if (_ser_port.is_open())
	{
		//Start io-service in a background thread.
		//boost::bind binds the ioservice instance
		//with the method call
		_t = boost::thread(
				boost::bind(
						&boost::asio::io_service::run,
						&_io));

		start_receive();
	}

	return _ser_port.is_open();
}

void SerialHandler::start_receive() {
	using namespace boost::asio;
	_ser_port.async_read_some(
			boost::asio::buffer(_buf,MAX_ELKA_MSG_LEN),
			boost::bind(&SerialHandler::data_cb,
				this, _1, _2));
}

void SerialHandler::send(uint8_t *tx_buffer, uint8_t len) {
	boost::asio::write(_ser_port, boost::asio::buffer(tx_buffer, len));
}

int SerialHandler::write_elka_msg(elka_msg_s &elka_msg) {
	return ELKA_ERROR;
}

int SerialHandler::write_elka_msg(elka_msg_ack_s &elka_msg) {
	return ELKA_ERROR;
}

void SerialHandler::data_cb(const boost::system::error_code& e,
	std::size_t size) {

	if (!e)
	{
		//memcpy(buf_repr, _buf, size);
		LOG_INFO("Received %d bytes:", (int)size);
    print_uint8_array(_buf, (uint16_t)size);
	}

	start_receive();
}
