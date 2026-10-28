#pragma once

// RS485 link to the host on Serial1.
//
// Requests: '#' <command> '\n'
// Replies:  lines ending in "\r\n", terminated by "<EOL>"
//
//   DATA  -> <ACK><DATA>, one "NAME(+a, +b, +c)<$>" line per live sensor, <EOL>
//   INIT  -> <ACK><INIT>(ADDED=<SENSOR_COUNT>,BAD=0)<EOL>   (sensors are fixed, args ignored)
//   OFFS  -> <ACK><OFFS>(OK)<EOL>              (offsets are fixed, args ignored)
//   PING  -> <ACK><UNKO>(PONG-PONG)<EOL>
//   HRST / SRST -> reboot
//   other -> <ACK><ERRR>(UNKNOWN)<EOL>

void commsBegin();

// Runs a command (without the leading '#') as if the host sent it, but prints the
// reply to USB serial instead of the bus. Used by debug mode.
void commsRunLocal(const char* cmd);

// Call often from loop(). Reads incoming bytes and answers complete requests.
void commsPoll();
