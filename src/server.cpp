/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * Server functions
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "OpenSprinkler.h"
#include "etherport.h"
#include "mqtt.h"
#include "program.h"
#include "server.h"
#include "weather.h"

#include <stdarg.h>
#include <stdlib.h>

extern EthernetClient *m_client;
#define handle_return(x)                                                       \
  {                                                                            \
    return_code = x;                                                           \
    return;                                                                    \
  }

extern char ether_buffer[];
extern char tmp_buffer[];
extern OpenSprinkler os;
extern ProgramData pd;
extern ulong flow_count;

static byte return_code;
static char *get_buffer = NULL;

BufferFiller bfill;

void schedule_all_stations(ulong curr_time);
void turn_off_station(byte sid, ulong curr_time);
void process_dynamic_events(ulong curr_time);
void check_weather(time_t curr_time);
void log_statistics(time_t curr_time);
void delete_log(char *name);
void reset_all_stations_immediate();
void reset_all_stations();
void make_logfile_name(char *name);

/* Check available space (number of bytes) in the Ethernet buffer */
int available_ether_buffer() {
  return ETHER_BUFFER_SIZE - (int)bfill.position();
}

// Define return error code
#define HTML_OK 0x00
#define HTML_SUCCESS 0x01
#define HTML_UNAUTHORIZED 0x02
#define HTML_MISMATCH 0x03
#define HTML_DATA_MISSING 0x10
#define HTML_DATA_OUTOFBOUND 0x11
#define HTML_DATA_FORMATERROR 0x12
#define HTML_RFCODE_ERROR 0x13
#define HTML_PAGE_NOT_FOUND 0x20
#define HTML_NOT_PERMITTED 0x30
#define HTML_UPLOAD_FAILED 0x40
#define HTML_REDIRECT_HOME 0xFF

static const char html200OK[] = "HTTP/1.1 200 OK\r\n";

static const char htmlNoCache[] =
    "Cache-Control: max-age=0, no-cache, no-store, must-revalidate\r\n";

static const char htmlContentHTML[] = "Content-Type: text/html\r\n";

static const char htmlAccessControl[] = "Access-Control-Allow-Origin: *\r\n";

static const char htmlContentJSON[] = "Content-Type: application/json\r\n"
                                      "Connection: close\r\n";

static const char htmlMobileHeader[] =
    "<meta name=\"viewport\" "
    "content=\"width=device-width,initial-scale=1.0,minimum-scale=1.0,user-"
    "scalable=no\">\r\n";

static const char htmlReturnHome[] =
    "<script>window.location=\"/\";</script>\n";

void print_html_standard_header() {
  bfill.emit_p(PSTR("$F$F$F$F\r\n"), html200OK, htmlContentHTML, htmlNoCache,
               htmlAccessControl);
}

void print_json_header(bool bracket = true) {
  bfill.emit_p(PSTR("$F$F$F$F\r\n"), html200OK, htmlContentJSON,
               htmlAccessControl, htmlNoCache);
  if (bracket)
    bfill.emit_p(PSTR("{"));
}

byte findKeyVal(const char *str, char *strbuf, uint16_t maxlen, const char *key,
                bool key_in_pgm = false, uint8_t *keyfound = NULL) {
  uint8_t found = 0;
  // case 2: otherwise, assume the key-val is stored in str
  uint16_t i = 0;
  const char *kp;
  kp = key;
  while (*str && *str != ' ' && *str != '\n' && found == 0) {
    if (*str == *kp) {
      kp++;
      if (*kp == '\0') {
        str++;
        kp = key;
        if (*str == '=') {
          found = 1;
        }
      }
    } else {
      kp = key;
    }
    str++;
  }
  if (found == 1) {
    // copy the value to a buffer and terminate it with '\0'
    while (*str && *str != ' ' && *str != '\n' && *str != '&' &&
           i < maxlen - 1) {
      *strbuf = *str;
      i++;
      str++;
      strbuf++;
    }
    if (!(*str) || *str == ' ' || *str == '\n' || *str == '&') {
      *strbuf = '\0';
    } else {
      found =
          0; // Ignore partial values i.e. value length is larger than maxlen
      i = 0;
    }
  }
  // return the length of the value
  if (keyfound)
    *keyfound = found;
  return (i);
}

void rewind_ether_buffer() {
  bfill = ether_buffer;
  ether_buffer[0] = 0;
}

void send_packet(bool final = false) {
  m_client->write((const uint8_t *)ether_buffer, strlen(ether_buffer));
  if (final) {
    m_client->stop();
  }
  rewind_ether_buffer();
  return;
}

char dec2hexchar(byte dec) {
  if (dec < 10)
    return '0' + dec;
  else
    return 'A' + (dec - 10);
}

void server_json_stations_attrib(const char *name, byte *attrib) {
  bfill.emit_p(PSTR("\"$F\":["), name);
  for (byte i = 0; i < os.nboards; i++) {
    bfill.emit_p(PSTR("$D"), attrib[i]);
    if (i != os.nboards - 1)
      bfill.emit_p(PSTR(","));
  }
  bfill.emit_p(PSTR("],"));
}

void server_json_stations_main() {
  server_json_stations_attrib(PSTR("masop"), os.attrib_mas);
  server_json_stations_attrib(PSTR("masop2"), os.attrib_mas2);
  server_json_stations_attrib(PSTR("ignore_rain"), os.attrib_igrd);
  server_json_stations_attrib(PSTR("ignore_sn1"), os.attrib_igs);
  server_json_stations_attrib(PSTR("ignore_sn2"), os.attrib_igs2);
  server_json_stations_attrib(PSTR("stn_dis"), os.attrib_dis);
  server_json_stations_attrib(PSTR("stn_seq"), os.attrib_seq);
  server_json_stations_attrib(PSTR("stn_spe"), os.attrib_spe);

  bfill.emit_p(PSTR("\"snames\":["));
  byte sid;
  for (sid = 0; sid < os.nstations; sid++) {
    os.get_station_name(sid, tmp_buffer);
    bfill.emit_p(PSTR("\"$S\""), tmp_buffer);
    if (sid != os.nstations - 1)
      bfill.emit_p(PSTR(","));
    if (available_ether_buffer() <= 0) {
      send_packet();
    }
  }
  bfill.emit_p(PSTR("],\"maxlen\":$D}"), STATION_NAME_SIZE);
}

/** Output stations data */
void server_json_stations() {
  print_json_header();
  server_json_stations_main();
  handle_return(HTML_OK);
}

/** Output station special attribute */
void server_json_station_special() {

  byte sid;
  byte comma = 0;
  StationData *data = (StationData *)tmp_buffer;
  print_json_header();
  for (sid = 0; sid < os.nstations; sid++) {
    if (os.get_station_type(sid) !=
        STN_TYPE_STANDARD) { // check if this is a special station
      os.get_station_data(sid, data);
      if (comma)
        bfill.emit_p(PSTR(","));
      else {
        comma = 1;
      }
      bfill.emit_p(PSTR("\"$D\":{\"st\":$D,\"sd\":\"$S\"}"), sid, data->type,
                   data->sped);
    }
    if (available_ether_buffer() <= 0) {
      send_packet();
    }
  }
  bfill.emit_p(PSTR("}"));
  handle_return(HTML_OK);
}

void server_change_stations_attrib(char *p, char header, byte *attrib) {
  char tbuf2[5] = {0, 0, 0, 0, 0};
  byte bid;
  tbuf2[0] = header;
  for (bid = 0; bid < os.nboards; bid++) {
    itoa(bid, tbuf2 + 1, 10);
    if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, tbuf2)) {
      attrib[bid] = atoi(tmp_buffer);
    }
  }
}

/**Change Station Name and Attributes
 * Command: /cs?s?=x&m?=x&i?=x&n?=x&d?=x
 *
 * s?: station name (? is station index, starting from 0)
 * m?: master operation bit field (? is board index, starting from 0)
 * i?: ignore rain bit field
 * n?: master2 operation bit field
 * d?: disable sation bit field
 * q?: station sequeitnal bit field
 * p?: station special flag bit field
 */
void server_change_stations() {
  char *p = get_buffer;

  byte sid;
  char tbuf2[5] = {'s', 0, 0, 0, 0};
  // process station names
  for (sid = 0; sid < os.nstations; sid++) {
    itoa(sid, tbuf2 + 1, 10);
    if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, tbuf2)) {
      urlDecode(tmp_buffer);
      os.set_station_name(sid, tmp_buffer);
    }
  }

  server_change_stations_attrib(p, 'm', os.attrib_mas);  // master1
  server_change_stations_attrib(p, 'i', os.attrib_igrd); // ignore rain delay
  server_change_stations_attrib(p, 'j', os.attrib_igs);  // ignore sensor1
  server_change_stations_attrib(p, 'k', os.attrib_igs2); // ignore sensor2
  server_change_stations_attrib(p, 'n', os.attrib_mas2); // master2
  server_change_stations_attrib(p, 'd', os.attrib_dis);  // disable
  server_change_stations_attrib(p, 'q', os.attrib_seq);  // sequential
  server_change_stations_attrib(p, 'p', os.attrib_spe);  // special

  /* handle special data */
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("sid"), true)) {
    sid = atoi(tmp_buffer);
    if (sid < 0 || sid > os.nstations)
      handle_return(HTML_DATA_OUTOFBOUND);
    if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("st"), true) &&
        findKeyVal(p, tmp_buffer + 1, TMP_BUFFER_SIZE - 1, PSTR("sd"), true)) {

      tmp_buffer[0] -= '0';
      tmp_buffer[STATION_SPECIAL_DATA_SIZE] = 0;

      // only process GPIO and HTTP stations for OS 2.3, above, and OSPi
      if (tmp_buffer[0] == STN_TYPE_GPIO) {
        // check that pin does not clash with OSPi pins
        byte gpio = (tmp_buffer[1] - '0') * 10 + tmp_buffer[2] - '0';
        byte activeState = tmp_buffer[3] - '0';

        byte gpioList[] = PIN_FREE_LIST;
        bool found = false;
        for (byte i = 0; i < sizeof(gpioList) && found == false; i++) {
          if (gpioList[i] == gpio)
            found = true;
        }
        if (!found || activeState > 1)
          handle_return(HTML_DATA_OUTOFBOUND);
      } else if (tmp_buffer[0] == STN_TYPE_HTTP) {
        urlDecode(tmp_buffer + 1);
        if (strlen(tmp_buffer + 1) > sizeof(HTTPStationData)) {
          handle_return(HTML_DATA_OUTOFBOUND);
        }
      }
      // write spe data
      file_write_block(STATIONS_FILENAME, tmp_buffer,
                       (uint32_t)sid * sizeof(StationData) +
                           offsetof(StationData, type),
                       STATION_SPECIAL_DATA_SIZE + 1);

    } else {

      handle_return(HTML_DATA_MISSING);
    }
  }

  os.attribs_save();

  handle_return(HTML_SUCCESS);
}

/** Parse one number from a comma separate list */
uint16_t parse_listdata(char **p) {
  char *pv;
  int i = 0;
  tmp_buffer[i] = 0;
  // copy to tmp_buffer until a non-number is encountered
  for (pv = (*p); pv < (*p) + 10; pv++) {
    if ((*pv) == '-' || (*pv) == '+' || ((*pv) >= '0' && (*pv) <= '9'))
      tmp_buffer[i++] = (*pv);
    else
      break;
  }
  tmp_buffer[i] = 0;
  *p = pv + 1;
  return (uint16_t)atol(tmp_buffer);
}

void manual_start_program(byte, byte);
/** Manual start program
 * Command: /mp?pid=xxx&uwt=xxx
 *
 * pid: program index (0 refers to the first program)
 * uwt: use weather (i.e. watering percentage)
 */
void server_manual_program() {
  char *p = get_buffer;

  if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("pid"), true))
    handle_return(HTML_DATA_MISSING);

  int pid = atoi(tmp_buffer);
  if (pid < 0 || pid >= pd.nprograms) {
    handle_return(HTML_DATA_OUTOFBOUND);
  }

  byte uwt = 0;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("uwt"), true)) {
    if (tmp_buffer[0] == '1')
      uwt = 1;
  }

  // reset all stations and prepare to run one-time program
  reset_all_stations_immediate();

  manual_start_program(pid + 1, uwt);

  handle_return(HTML_SUCCESS);
}

/**
 * Change run-once program
 * Command: /cr?t=[x,x,x...]
 *
 * t:  station water time
 */
void server_change_runonce() {
  char *p = get_buffer;

  // decode url first
  if (p)
    urlDecode(p);
  // search for the start of t=[
  char *pv;
  boolean found = false;
  for (pv = p; (*pv) != 0 && pv < p + 100; pv++) {
    if (strncmp(pv, "t=[", 3) == 0) {
      found = true;
      break;
    }
  }
  if (!found)
    handle_return(HTML_DATA_MISSING);
  pv += 3;

  // reset all stations and prepare to run one-time program
  reset_all_stations_immediate();

  byte sid, bid, s;
  uint16_t dur;
  boolean match_found = false;
  for (sid = 0; sid < os.nstations; sid++) {
    dur = parse_listdata(&pv);
    bid = sid >> 3;
    s = sid & 0x07;
    // if non-zero duration is given
    // and if the station has not been disabled
    if (dur > 0 && !(os.attrib_dis[bid] & (1 << s))) {
      RuntimeQueueStruct *q = pd.enqueue();
      if (q) {
        q->st = 0;
        q->dur = water_time_resolve(dur);
        q->pid = 254;
        q->sid = sid;
        match_found = true;
      }
    }
  }
  if (match_found) {
    schedule_all_stations(os.now_tz());
    handle_return(HTML_SUCCESS);
  }

  handle_return(HTML_DATA_MISSING);
}

/**
 * Delete a program
 * Command: /dp?pid=xxx
 *
 * pid:program index (-1 will delete all programs)
 */
void server_delete_program() {
  char *p = get_buffer;
  if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("pid"), true))
    handle_return(HTML_DATA_MISSING);

  int pid = atoi(tmp_buffer);
  if (pid == -1) {
    pd.eraseall();
  } else if (pid < pd.nprograms) {
    pd.del(pid);
  } else {
    handle_return(HTML_DATA_OUTOFBOUND);
  }

  handle_return(HTML_SUCCESS);
}

/**
 * Move up a program
 * Command: /up?pid=xxx
 *
 * pid: program index (must be 1 or larger, because we can't move up program 0)
 */
void server_moveup_program() {
  char *p = get_buffer;

  if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("pid"), true))
    handle_return(HTML_DATA_MISSING);

  int pid = atoi(tmp_buffer);
  if (!(pid >= 1 && pid < pd.nprograms))
    handle_return(HTML_DATA_OUTOFBOUND);

  pd.moveup(pid);

  handle_return(HTML_SUCCESS);
}

/**
 * Change a program
 * Command:
 * /cp?pid=x&v=[flag,days0,days1,[start0,start1,start2,start3],[dur0,dur1,dur2..]]&name=x
 *
 * pid:		program index
 * flag:	program flag
 * start?:up to 4 start times
 * dur?:	station water time
 * name:	program name
 */
const char _str_program[] = "Program ";
void server_change_program() {
  char *p = get_buffer;

  byte i;

  ProgramStruct prog;

  // parse program index
  if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("pid"), true))
    handle_return(HTML_DATA_MISSING);

  int pid = atoi(tmp_buffer);
  if (!(pid >= -1 && pid < pd.nprograms))
    handle_return(HTML_DATA_OUTOFBOUND);

  // check if "en" parameter is present
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("en"), true)) {
    if (pid < 0)
      handle_return(HTML_DATA_OUTOFBOUND);
    pd.set_flagbit(pid, PROGRAMSTRUCT_EN_BIT, (tmp_buffer[0] == '0') ? 0 : 1);
    handle_return(HTML_SUCCESS);
  }

  // check if "uwt" parameter is present
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("uwt"), true)) {
    if (pid < 0)
      handle_return(HTML_DATA_OUTOFBOUND);
    pd.set_flagbit(pid, PROGRAMSTRUCT_UWT_BIT, (tmp_buffer[0] == '0') ? 0 : 1);
    handle_return(HTML_SUCCESS);
  }

  // parse program name
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("name"), true)) {
    urlDecode(tmp_buffer);
    strncpy(prog.name, tmp_buffer, PROGRAM_NAME_SIZE);
  } else {
    strcpy_P(prog.name, _str_program);
    itoa((pid == -1) ? (pd.nprograms + 1) : (pid + 1), prog.name + 8, 10);
  }

  // do a full string decoding
  if (p)
    urlDecode(p);

  // parse ad-hoc v=[...
  // search for the start of v=[
  char *pv;
  boolean found = false;

  for (pv = p; (*pv) != 0 && pv < p + 100; pv++) {
    if (strncmp(pv, "v=[", 3) == 0) {
      found = true;
      break;
    }
  }

  if (!found)
    handle_return(HTML_DATA_MISSING);
  pv += 3;

  // parse headers
  *(char *)(&prog) = parse_listdata(&pv);
  prog.days[0] = parse_listdata(&pv);
  prog.days[1] = parse_listdata(&pv);
  // parse start times
  pv++; // this should be a '['
  for (i = 0; i < MAX_NUM_STARTTIMES; i++) {
    prog.starttimes[i] = parse_listdata(&pv);
  }
  pv++; // this should be a ','
  pv++; // this should be a '['
  for (i = 0; i < os.nstations; i++) {
    uint16_t pre = parse_listdata(&pv);
    prog.durations[i] = pre;
  }
  pv++; // this should be a ']'
  pv++; // this should be a ']'
  // parse program name

  // i should be equal to os.nstations at this point
  for (; i < MAX_NUM_STATIONS; i++) {
    prog.durations[i] = 0; // clear unused field
  }

  // process interval day remainder (relative-> absolute)
  if (prog.type == PROGRAM_TYPE_INTERVAL && prog.days[1] >= 1) {
    pd.drem_to_absolute(prog.days);
  }

  if (pid == -1) {
    if (!pd.add(&prog))
      handle_return(HTML_DATA_OUTOFBOUND);
  } else {
    if (!pd.modify(pid, &prog))
      handle_return(HTML_DATA_OUTOFBOUND);
  }
  handle_return(HTML_SUCCESS);
}

void server_json_options_main() {
  byte oid;
  for (oid = 0; oid < NUM_IOPTS; oid++) {
    int32_t v = os.iopts[oid];
    if (oid == IOPT_MASTER_OFF_ADJ || oid == IOPT_MASTER_OFF_ADJ_2 ||
        oid == IOPT_MASTER_ON_ADJ || oid == IOPT_MASTER_ON_ADJ_2 ||
        oid == IOPT_STATION_DELAY_TIME) {
      v = water_time_decode_signed(v);
    }

    if (oid == IOPT_BOOST_TIME)
      continue;

    // each json name takes 5 characters
    strncpy_P0(tmp_buffer, iopt_json_names + oid * 5, 5);
    bfill.emit_p(PSTR("\"$S\":$D"), tmp_buffer, v);
    if (oid != NUM_IOPTS - 1)
      bfill.emit_p(PSTR(","));
  }

  bfill.emit_p(PSTR(",\"dexp\":$D,\"mexp\":$D,\"hwt\":$D}"), os.detect_exp(),
               MAX_EXT_BOARDS, os.hw_type);
}

/** Output Options */
void server_json_options() {
  print_json_header();
  server_json_options_main();
  handle_return(HTML_OK);
}

void server_json_programs_main() {

  bfill.emit_p(PSTR("\"nprogs\":$D,\"nboards\":$D,\"mnp\":$D,\"mnst\":$D,"
                    "\"pnsize\":$D,\"pd\":["),
               pd.nprograms, os.nboards, MAX_NUM_PROGRAMS, MAX_NUM_STARTTIMES,
               PROGRAM_NAME_SIZE);
  byte pid, i;
  ProgramStruct prog;
  for (pid = 0; pid < pd.nprograms; pid++) {
    pd.read(pid, &prog);
    if (prog.type == PROGRAM_TYPE_INTERVAL && prog.days[1] >= 1) {
      pd.drem_to_relative(prog.days);
    }

    byte bytedata = *(char *)(&prog);
    bfill.emit_p(PSTR("[$D,$D,$D,["), bytedata, prog.days[0], prog.days[1]);
    // start times data
    for (i = 0; i < (MAX_NUM_STARTTIMES - 1); i++) {
      bfill.emit_p(PSTR("$D,"), prog.starttimes[i]);
    }
    bfill.emit_p(PSTR("$D],["), prog.starttimes[i]); // this is the last element
    // station water time
    for (i = 0; i < os.nstations - 1; i++) {
      bfill.emit_p(PSTR("$L,"), (unsigned long)prog.durations[i]);
    }
    bfill.emit_p(PSTR("$L],\""),
                 (unsigned long)prog.durations[i]); // this is the last element
    // program name
    strncpy(tmp_buffer, prog.name, PROGRAM_NAME_SIZE);
    tmp_buffer[PROGRAM_NAME_SIZE] = 0; // make sure the string ends
    bfill.emit_p(PSTR("$S"), tmp_buffer);
    if (pid != pd.nprograms - 1) {
      bfill.emit_p(PSTR("\"],"));
    } else {
      bfill.emit_p(PSTR("\"]"));
    }
    // push out a packet if available
    // buffer size is getting small
    if (available_ether_buffer() <= 0) {
      send_packet();
    }
  }
  bfill.emit_p(PSTR("]}"));
}

/** Output program data */
void server_json_programs() {

  print_json_header();
  server_json_programs_main();
  handle_return(HTML_OK);
}

/** Output script url form */
void server_view_scripturl() {

  print_html_standard_header();
  bfill.emit_p(
      PSTR("<form name=of action=cu method=get><table "
           "cellspacing=\"12\"><tr><td><b>JavaScript</b>:</td><td><input "
           "type=text size=40 maxlength=40 value=\"$O\" "
           "name=jsp></td></tr><tr><td>Default:</td><td>$S</td></"
           "tr><tr><td><b>Weather</b>:</td><td><input type=text size=40 "
           "maxlength=40 value=\"$O\" "
           "name=wsp></td></tr><tr><td>Default:</td><td>$S</td></"
           "tr><tr><td><b>Password</b>:</td><td><input type=password size=32 "
           "name=pw> <input type=submit></td></tr></table></form><script "
           "src=https://ui.opensprinkler.com/js/hasher.js></script>"),
      SOPT_JAVASCRIPTURL, DEFAULT_JAVASCRIPT_URL, SOPT_WEATHERURL,
      DEFAULT_WEATHER_URL);
  handle_return(HTML_OK);
}

void server_json_controller_main() {
  byte bid, sid;
  ulong curr_time = os.now_tz();
  bfill.emit_p(
      PSTR("\"devt\":$L,\"nbrd\":$D,\"en\":$D,\"sn1\":$D,\"sn2\":$D,\"rd\":$D,"
           "\"rdst\":$L,"
           "\"sunrise\":$D,\"sunset\":$D,\"eip\":$L,\"lwc\":$L,\"lswc\":$L,"
           "\"lupt\":$L,\"lrbtc\":$D,\"lrun\":[$D,$D,$D,$L],"),
      curr_time, os.nboards, os.status.enabled, os.status.sensor1_active,
      os.status.sensor2_active, os.status.rain_delayed, os.nvdata.rd_stop_time,
      os.nvdata.sunrise_time, os.nvdata.sunset_time, os.nvdata.external_ip,
      os.checkwt_lasttime, os.checkwt_success_lasttime, os.powerup_lasttime,
      os.last_reboot_cause, pd.lastrun.station, pd.lastrun.program,
      pd.lastrun.duration, pd.lastrun.endtime);

  bfill.emit_p(PSTR("\"loc\":\"$O\",\"jsp\":\"$O\",\"wsp\":\"$O\",\"wto\":{$O},"
                    "\"mqtt\":{$O},\"wtdata\":$S,\"wterr\":$D,"),
               SOPT_LOCATION, SOPT_JAVASCRIPTURL, SOPT_WEATHERURL,
               SOPT_WEATHER_OPTS, SOPT_MQTT_OPTS,
               strlen(wt_rawData) == 0 ? "{}" : wt_rawData, wt_errCode);

  if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW) {
    bfill.emit_p(PSTR("\"flcrt\":$L,\"flwrt\":$D,"), os.flowcount_rt,
                 FLOWCOUNT_RT_WINDOW);
  }

  bfill.emit_p(PSTR("\"sbits\":["));
  // print sbits
  for (bid = 0; bid < os.nboards; bid++)
    bfill.emit_p(PSTR("$D,"), os.station_bits[bid]);
  bfill.emit_p(PSTR("0],\"ps\":["));
  // print ps
  for (sid = 0; sid < os.nstations; sid++) {
    // if available ether buffer is getting small
    // send out a packet
    if (available_ether_buffer() <= 0) {
      send_packet();
    }
    unsigned long rem = 0;
    byte qid = pd.station_qid[sid];
    RuntimeQueueStruct *q = pd.queue + qid;
    if (qid < 255) {
      rem = (curr_time >= q->st) ? (q->st + q->dur - curr_time) : q->dur;
      if (rem > 65535)
        rem = 0;
    }
    bfill.emit_p(PSTR("[$D,$L,$L]"), (qid < 255) ? q->pid : 0, rem,
                 (qid < 255) ? q->st : 0);
    bfill.emit_p((sid < os.nstations - 1) ? PSTR(",") : PSTR("]"));
  }

  bfill.emit_p(PSTR("}"));
}

/** Output controller variables in json */
void server_json_controller() {
  print_json_header();
  server_json_controller_main();
  handle_return(HTML_OK);
}

/** Output homepage */
void server_home() {

  print_html_standard_header();

  bfill.emit_p(
      PSTR("<!DOCTYPE html>\n<html>\n<head>\n$F</head>\n<body>\n<script>"),
      htmlMobileHeader);
  // send server variables and javascript packets
  bfill.emit_p(PSTR("var ver=$D,ipas=1;</script>\n"), OS_FW_VERSION);

  bfill.emit_p(PSTR("<script src=\"$O/home.js\"></script>\n</body>\n</html>"),
               SOPT_JAVASCRIPTURL);

  handle_return(HTML_OK);
}

/**
 * Change controller variables
 * Command: /cv?rsn=x&rbt=x&en=x&rd=x&re=x&ap=x
 *
 * rsn: reset all stations (0 or 1)
 * rbt: reboot controller (0 or 1)
 * en:	enable (0 or 1)
 * rd:	rain delay hours (0 turns off rain delay)
 * re:	remote extension mode
 */
void server_change_values() {
  char *p = get_buffer;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("rsn"), true)) {
    reset_all_stations();
  }

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("rbt"), true) &&
      atoi(tmp_buffer) > 0) {
    print_html_standard_header();
    send_packet(true);
    os.reboot_dev(REBOOT_CAUSE_WEB);
  }

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("en"), true)) {
    if (tmp_buffer[0] == '1' && !os.status.enabled)
      os.enable();
    else if (tmp_buffer[0] == '0' && os.status.enabled)
      os.disable();
  }

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("rd"), true)) {
    int rd = atoi(tmp_buffer);
    if (rd > 0) {
      os.nvdata.rd_stop_time = os.now_tz() + (unsigned long)rd * 3600;
      os.raindelay_start();
    } else if (rd == 0) {
      os.raindelay_stop();
    } else
      handle_return(HTML_DATA_OUTOFBOUND);
  }

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("re"), true)) {
    if (tmp_buffer[0] == '1' && !os.iopts[IOPT_REMOTE_EXT_MODE]) {
      os.iopts[IOPT_REMOTE_EXT_MODE] = 1;
      os.iopts_save();
    } else if (tmp_buffer[0] == '0' && os.iopts[IOPT_REMOTE_EXT_MODE]) {
      os.iopts[IOPT_REMOTE_EXT_MODE] = 0;
      os.iopts_save();
    }
  }

  handle_return(HTML_SUCCESS);
}

// remove spaces from a string
void string_remove_space(char *src) {
  char *dst = src;
  while (1) {
    if (*src != ' ') {
      *dst++ = *src;
    }
    if (*src == 0)
      break;
    src++;
  }
}

/**
 * Change script url
 * Command: /cu?jsp=x
 *
 * jsp: Javascript path
 */
void server_change_scripturl() {
  char *p = get_buffer;

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("jsp"), true)) {
    urlDecode(tmp_buffer);
    tmp_buffer[TMP_BUFFER_SIZE] =
        0; // make sure we don't exceed the maximum size
    // trim unwanted space characters
    string_remove_space(tmp_buffer);
    os.sopt_save(SOPT_JAVASCRIPTURL, tmp_buffer);
  }
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("wsp"), true)) {
    urlDecode(tmp_buffer);
    tmp_buffer[TMP_BUFFER_SIZE] = 0;
    string_remove_space(tmp_buffer);
    os.sopt_save(SOPT_WEATHERURL, tmp_buffer);
  }
  handle_return(HTML_REDIRECT_HOME);
}

/**
 * Change options
 * Command: /co?o?=x&loc=x&ttt=x
 *
 * o?:	option name (? is option index)
 * loc: location
 * ttt: manual time
 */
void server_change_options() {
  char *p = get_buffer;

  // temporarily save some old options values
  bool weather_change = false;
  bool sensor_change = false;

  // !!! p and bfill share the same buffer, so don't write
  // to bfill before you are done analyzing the buffer !!!
  // process option values
  byte err = 0;
  byte prev_value;
  byte max_value;
  for (byte oid = 0; oid < NUM_IOPTS; oid++) {

    // skip options that cannot be set through /co command
    if (oid == IOPT_FW_VERSION || oid == IOPT_HW_VERSION ||
        oid == IOPT_DEVICE_ENABLE || oid == IOPT_FW_MINOR ||
        oid == IOPT_REMOTE_EXT_MODE || oid == IOPT_RESET ||
        oid == IOPT_WIFI_MODE)
      continue;
    prev_value = os.iopts[oid];
    max_value = pgm_read_byte(iopt_max + oid);

    // will no longer support oxx option names
    // json name only
    char tbuf2[6];
    strncpy_P0(tbuf2, iopt_json_names + oid * 5, 5);
    if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, tbuf2)) {
      int32_t v = atol(tmp_buffer);
      if (oid == IOPT_MASTER_OFF_ADJ || oid == IOPT_MASTER_OFF_ADJ_2 ||
          oid == IOPT_MASTER_ON_ADJ || oid == IOPT_MASTER_ON_ADJ_2 ||
          oid == IOPT_STATION_DELAY_TIME) {
        v = water_time_encode_signed(v);
      } // encode station delay time
      if (oid == IOPT_BOOST_TIME) {
        v >>= 2;
      }
      if (v >= 0 && v <= max_value) {
        os.iopts[oid] = v;
      } else {
        err = 1;
      }
    }

    if (os.iopts[oid] != prev_value) { // if value has changed
      if (oid == IOPT_USE_WEATHER)
        weather_change = true;
      if (oid >= IOPT_SENSOR1_TYPE && oid <= IOPT_SENSOR2_OFF_DELAY)
        sensor_change = true;
    }
  }

  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("loc"), true)) {
    urlDecode(tmp_buffer);
    if (os.sopt_save(SOPT_LOCATION,
                     tmp_buffer)) { // if location string has changed
      weather_change = true;
    }
  }
  uint8_t keyfound = 0;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("wto"), true)) {
    urlDecode(tmp_buffer);
    if (os.sopt_save(SOPT_WEATHER_OPTS, tmp_buffer)) {
      weather_change = true; // if wto has changed
    }
    // DEBUG_PRINTLN(os.sopt_load(SOPT_WEATHER_OPTS));
  }

  keyfound = 0;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("mqtt"), true,
                 &keyfound)) {
    urlDecode(tmp_buffer);
    os.sopt_save(SOPT_MQTT_OPTS, tmp_buffer);
    os.status.req_mqtt_restart = true;
  } else if (keyfound) {
    tmp_buffer[0] = 0;
    os.sopt_save(SOPT_MQTT_OPTS, tmp_buffer);
    os.status.req_mqtt_restart = true;
  }

  if (err)
    handle_return(HTML_DATA_OUTOFBOUND);

  os.iopts_save();

  if (weather_change) {
    os.iopts[IOPT_WATER_PERCENTAGE] = 100; // reset watering percentage to 100%
    wt_rawData[0] = 0;                     // reset wt_rawData and errCode
    wt_errCode = HTTP_RQT_NOT_RECEIVED;
    os.checkwt_lasttime = 0; // force weather update
  }

  if (sensor_change) {
    os.sensor_resetall();
  }

  handle_return(HTML_SUCCESS);
}

void server_json_status_main() {
  bfill.emit_p(PSTR("\"sn\":["));
  byte sid;

  for (sid = 0; sid < os.nstations; sid++) {
    bfill.emit_p(PSTR("$D"), (os.station_bits[(sid >> 3)] >> (sid & 0x07)) & 1);
    if (sid != os.nstations - 1)
      bfill.emit_p(PSTR(","));
  }
  bfill.emit_p(PSTR("],\"nstations\":$D}"), os.nstations);
}

/** Output station status */
void server_json_status() {
  print_json_header();
  server_json_status_main();
  handle_return(HTML_OK);
}

/**
 * Test station (previously manual operation)
 * Command: /cm?sid=x&en=x&t=x
 *
 * sid:station index (starting from 0)
 * en: enable (0 or 1)
 * t:  timer (required if en=1)
 */
void server_change_manual() {
  char *p = get_buffer;

  int sid = -1;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("sid"), true)) {
    sid = atoi(tmp_buffer);
    if (sid < 0 || sid >= os.nstations)
      handle_return(HTML_DATA_OUTOFBOUND);
  } else {
    handle_return(HTML_DATA_MISSING);
  }

  byte en = 0;
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("en"), true)) {
    en = atoi(tmp_buffer);
  } else {
    handle_return(HTML_DATA_MISSING);
  }

  uint16_t timer = 0;
  unsigned long curr_time = os.now_tz();
  if (en) { // if turning on a station, must provide timer
    if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("t"), true)) {
      timer = (uint16_t)atol(tmp_buffer);
      if (timer == 0 || timer > 64800) {
        handle_return(HTML_DATA_OUTOFBOUND);
      }
      // schedule manual station
      // skip if the station is a master station
      // (because master cannot be scheduled independently)
      if ((os.status.mas == sid + 1) || (os.status.mas2 == sid + 1))
        handle_return(HTML_NOT_PERMITTED);

      RuntimeQueueStruct *q = NULL;
      byte sqi = pd.station_qid[sid];
      // check if the station already has a schedule
      if (sqi != 0xFF) { // if so, we will overwrite the schedule
        q = pd.queue + sqi;
      } else { // otherwise create a new queue element
        q = pd.enqueue();
      }
      // if the queue is not full
      if (q) {
        q->st = 0;
        q->dur = timer;
        q->sid = sid;
        q->pid = 99; // testing stations are assigned program index 99
        schedule_all_stations(curr_time);
      } else {
        handle_return(HTML_NOT_PERMITTED);
      }
    } else {
      handle_return(HTML_DATA_MISSING);
    }
  } else { // turn off station
    turn_off_station(sid, curr_time);
  }
  handle_return(HTML_SUCCESS);
}

/**
 * Get log data
 * Command: /jl?start=x&end=x&hist=x&type=x
 *
 * hist:	history (past n days)
 *				when hist is speceified, the start
 *				and end parameters below will be ignored
 * start: start time (epoch time)
 * end:		end time (epoch time)
 * type:	type of log records (optional)
 *				rs, rd, wl
 *				if unspecified, output all records
 */
void server_json_log() {

  char *p = get_buffer;

  unsigned int start, end;

  // past n day history
  if (findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("hist"), true)) {
    int hist = atoi(tmp_buffer);
    if (hist < 0 || hist > 365)
      handle_return(HTML_DATA_OUTOFBOUND);
    end = os.now_tz() / 86400L;
    start = end - hist;
  } else {
    if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("start"), true))
      handle_return(HTML_DATA_MISSING);

    start = strtoul(tmp_buffer, NULL, 0) / 86400L;

    if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("end"), true))
      handle_return(HTML_DATA_MISSING);

    end = strtoul(tmp_buffer, NULL, 0) / 86400L;

    // start must be prior to end, and can't retrieve more than 365 days of data
    if ((start > end) || (end - start) > 365)
      handle_return(HTML_DATA_OUTOFBOUND);
  }

  // extract the type parameter
  char type[4] = {0};
  bool type_specified = false;
  if (findKeyVal(p, type, 4, PSTR("type"), true))
    type_specified = true;

  print_json_header(false);

  bfill.emit_p(PSTR("["));

  bool comma = 0;
  for (unsigned int i = start; i <= end; i++) {
    itoa(i, tmp_buffer, 10);
    make_logfile_name(tmp_buffer);

    FILE *file = fopen(tmp_buffer, "rb");
    if (!file)
      continue;

    int res;
    while (true) {
      if (fgets(tmp_buffer, TMP_BUFFER_SIZE, file)) {
        res = strlen(tmp_buffer);
      } else {
        res = 0;
      }
      if (res <= 0) {
        fclose(file);
        break;
      }
      // check record type
      // records are all in the form of [x,"xx",...]
      // where x is program index (>0) if this is a station record
      // and "xx" is the type name if this is a special record (e.g. wl, fl, rs)

      // search string until we find the first comma
      char *ptype = tmp_buffer;
      tmp_buffer[TMP_BUFFER_SIZE - 1] = 0; // make sure the search will end
      while (*ptype && *ptype != ',')
        ptype++;
      if (*ptype != ',')
        continue; // didn't find comma, move on
      ptype++;    // move past comma

      if (type_specified && strncmp(type, ptype + 1, 2))
        continue;
      // if type is not specified, output everything except "wl" and "fl"
      // records
      if (!type_specified &&
          (!strncmp("wl", ptype + 1, 2) || !strncmp("fl", ptype + 1, 2)))
        continue;
      // if this is the first record, do not print comma
      if (comma)
        bfill.emit_p(PSTR(","));
      else {
        comma = 1;
      }
      bfill.emit_p(PSTR("$S"), tmp_buffer);
      // if the available ether buffer size is getting small
      // push out a packet
      if (available_ether_buffer() <= 0) {
        send_packet();
      }
    }
  }

  bfill.emit_p(PSTR("]"));
  handle_return(HTML_OK);
}
/**
 * Delete log
 * Command: /dl?day=xxx
 *					/dl?day=all
 *
 * day:day (epoch time / 86400)
 * if day=all: delete all log files)
 */
void server_delete_log() {
  char *p = get_buffer;

  if (!findKeyVal(p, tmp_buffer, TMP_BUFFER_SIZE, PSTR("day"), true))
    handle_return(HTML_DATA_MISSING);

  delete_log(tmp_buffer);

  handle_return(HTML_SUCCESS);
}

/** Output all JSON data, including jc, jp, jo, js, jn */
void server_json_all() {
  print_json_header();
  bfill.emit_p(PSTR("\"settings\":{"));
  server_json_controller_main();
  send_packet();
  bfill.emit_p(PSTR(",\"programs\":{"));
  server_json_programs_main();
  send_packet();
  bfill.emit_p(PSTR(",\"options\":{"));
  server_json_options_main();
  send_packet();
  bfill.emit_p(PSTR(",\"status\":{"));
  server_json_status_main();
  send_packet();
  bfill.emit_p(PSTR(",\"stations\":{"));
  server_json_stations_main();
  bfill.emit_p(PSTR("}"));
  handle_return(HTML_OK);
}

typedef void (*URLHandler)(void);

/* Server function urls
 * To save RAM space, each GET command keyword is exactly
 * 2 characters long, with no ending 0
 * The order must exactly match the order of the
 * handler functions below
 */
const char _url_keys[] = "cv"
                         "jc"
                         "dp"
                         "cp"
                         "cr"
                         "mp"
                         "up"
                         "jp"
                         "co"
                         "jo"
                         "js"
                         "cm"
                         "cs"
                         "jn"
                         "je"
                         "jl"
                         "dl"
                         "su"
                         "cu"
                         "ja";

// Server function handlers
URLHandler urls[] = {
    server_change_values,        // cv
    server_json_controller,      // jc
    server_delete_program,       // dp
    server_change_program,       // cp
    server_change_runonce,       // cr
    server_manual_program,       // mp
    server_moveup_program,       // up
    server_json_programs,        // jp
    server_change_options,       // co
    server_json_options,         // jo
    server_json_status,          // js
    server_change_manual,        // cm
    server_change_stations,      // cs
    server_json_stations,        // jn
    server_json_station_special, // je
    server_json_log,             // jl
    server_delete_log,           // dl
    server_view_scripturl,       // su
    server_change_scripturl,     // cu
    server_json_all,             // ja
};

// handle Ethernet request

void handle_web_request(char *p) {
  rewind_ether_buffer();

  // assume this is a GET request
  // GET /xx?xxxx
  char *com = p + 5;
  char *dat = com + 3;

  if (com[0] == ' ') {
    server_home(); // home page handler
    send_packet(true);
  } else {
    // server funtion handlers
    byte i;
    for (i = 0; i < sizeof(urls) / sizeof(URLHandler); i++) {
      if (pgm_read_byte(_url_keys + 2 * i) == com[0] &&
          pgm_read_byte(_url_keys + 2 * i + 1) == com[1]) {

        int ret = HTML_UNAUTHORIZED;

        if (com[0] == 's' && com[1] == 'u') {
          get_buffer = dat;
          (urls[i])();
          ret = return_code;
        } else if ((com[0] == 'j' && com[1] == 'o') ||
                   (com[0] == 'j' && com[1] == 'a')) {
          get_buffer = dat;
          (urls[i])();
          ret = return_code;
        } else if (com[0] == 'd' && com[1] == 'b') {
          get_buffer = dat;
          (urls[i])();
          ret = return_code;
        } else {
          get_buffer = dat;
          (urls[i])();
          ret = return_code;
        }
        if (ret == -1) {
          if (m_client)
            m_client->stop();
          return;
        }
        switch (ret) {
        case HTML_OK:
          break;
        case HTML_REDIRECT_HOME:
          print_html_standard_header();
          bfill.emit_p(PSTR("$F"), htmlReturnHome);
          break;
        default:
          print_json_header();
          bfill.emit_p(PSTR("\"result\":$D}"), ret);
        }
        break;
      }
    }

    if (i == sizeof(urls) / sizeof(URLHandler)) {
      // no server funtion found
      print_json_header();
      bfill.emit_p(PSTR("\"result\":$D}"), HTML_PAGE_NOT_FOUND);
    }
    send_packet(true);
  }
}
