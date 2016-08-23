
/* @file ashtech protocol definitions */

#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "device.h"
#include "uart.h"
#include "ringbuffer.h"

#include "vehicle_gps_position.h"
#include "satellite_info.h"

#include "os/thread.h"

#ifndef RECV_BUFFER_SIZE
#define RECV_BUFFER_SIZE 512
#define SAT_INFO_MAX_SATELLITES  20
#endif


class ashtech : public device, public thread
{
	enum ashtech_decode_state_t {
		NME_DECODE_UNINIT,
		NME_DECODE_GOT_SYNC1,
		NME_DECODE_GOT_ASTERIKS,
		NME_DECODE_GOT_FIRST_CS_BYTE
	};


	s32 ashtechlog_fd;

	ashtech_decode_state_t	_decode_state;
	u8		_rx_buffer[RECV_BUFFER_SIZE];
	u16   	_rx_buffer_bytes;
	bool 	_parse_error; 		/** parse error flag */
	char 	*_parse_pos; 		/** parse position */

	bool	_gsv_in_progress;			/**< Indicates that gsv data parsing is in progress */
	/* s32     _satellites_count; 			**< Number of satellites info parsed. */
	u8 count;					/**< Number of satellites in satellite info */
	u8 svid[SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	u8 used[SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	u8 elevation[SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	u8 azimuth[SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	u8 snr[SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */

public:
	ashtech(PCSTR devname, s32 devid);
	~ashtech(void);

	s32             configure(unsigned &baudrate);
	s32             handle_message(s32 len);
	s32             parse_char(u8 b);
	/** Read s32 ashtech parameter */
	s32         read_int();
	/** Read float ashtech parameter */
	double       read_float();
	/** Read char ashtech parameter */
	char            read_char();

public:
    s32 probe(uart *puart);
    s32 remove(void);

public:
	uart *_puart;

	struct vehicle_gps_position_s _gps_position;
	struct satellite_info_s _satellite_info;
	ringbuffer	*_gps_position_reports;
	ringbuffer	*_satellite_info_reports;

public:
	s32 init(void);
	s32 reset(void);
	void measure(void);

public:
	virtual void run(void *parg);
};

#endif /* ASHTECH_H_ */
