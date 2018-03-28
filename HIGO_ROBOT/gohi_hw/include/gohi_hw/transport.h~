#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>

#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>


typedef std::vector<uint8_t> Buffer;

class Transport {
public:
	Transport(std::string url) :
		comm_url_(url),
		write_buffer_(),                                     
		read_buffer_()
	{
		//boost::shared_ptr<boost::asio::io_service> ios_;
		ios_ = boost::make_shared<boost::asio::io_service>();//new
	}

	virtual Buffer readBuffer() = 0;

	virtual void writeBuffer(Buffer &data) = 0;

	inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
	{
		return ios_;
	}

	bool initialize_ok()
	{
		return initialize_ok_;
	}

protected:
	// for communication location
	std::string comm_url_;
	std::queue<Buffer> write_buffer_;
	std::queue<Buffer> read_buffer_;

	bool initialize_ok_;

	// for boost asio service
	boost::shared_ptr<boost::asio::io_service> ios_;
};





#endif /* TRANSPORT_BASE_H_ */
