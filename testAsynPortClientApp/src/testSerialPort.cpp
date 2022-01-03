#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <map>

#include <epicsString.h>
#include <epicsThread.h>
#include <drvAsynSerialPort.h>
#include <drvAsynIPPort.h>
#include <asynPortClient.h>

#include <CLI/CLI.hpp>

static CLI::Validator CheckYesNo = CLI::IsMember({"Y", "N"}, CLI::ignore_case);
static CLI::Validator CheckNumber = CLI::Number;
static CLI::Validator CheckParity = CLI::IsMember({"odd", "even", "none", "mark", "space"});
static CLI::Validator CheckPurge = CLI::IsMember({"rxclear", "txclear", "rxabort", "txabort"});
static CLI::Validator CheckEscape = CLI::IsMember({"clrbreak", "clrdtr", "clrrts", "setbreak", "setdtr", "setrts", "setxoff", "setxon"});

static void addOption(CLI::App& app, std::map<std::string, std::string>& optMap, const std::string& opt, const std::string& desc, CLI::Validator validator)
{
    app.add_option(std::string("--") + opt, optMap[opt], desc)->check(validator);
}   

// testSerialPort [options] COMPORT [outputString] [outputEos] [inputEos]

// for details run
//
//     testSerialPort --help

// The command uses escaped characters as per printf/epicsStrnRawFromEscaped 
// e.g. for eurotherm the Stream device protocol \x05 (hex) would be written as \005 (octal)
//
//     testserialport COM7 "\0040011PV" "\005" "\003" --eventmask=0x1ff
//

int main(int argc, char *argv[])
{
  CLI::App app;
  std::map<std::string, std::string> optMap;

  std::string output_string, input_eos, output_eos, serial_port;
  int serial_report = -1;
  double wait = 0.0, timeout = DEFAULT_TIMEOUT;
  bool noread = false;

  app.add_option("--port,port", serial_port, "COM Port e.g. COM5")->mandatory();
  app.add_option("output_string", output_string, "Output string, may include escaped characters such as \\n or \\005");
  app.add_option("output_eos", output_eos, "Output EOS e.g. \\r\\n");
  app.add_option("input_eos", input_eos, "Input EOS e.g. \\r\\n");
  app.add_option("--report", serial_report, "Run device reports of this level (dbior)");
  app.add_flag("--noread", noread, "Do not read from port after write");
  app.add_option("--wait", wait, "Delay between write and read (s)");
  app.add_option("--timeout", timeout, "Read/write timeout (s)");
  
  addOption(app, optMap, "baud", "Baud Rate", CheckNumber);
  addOption(app, optMap, "bits", "Data Bits", CheckNumber);
  addOption(app, optMap, "parity", "Parity", CheckParity);
  addOption(app, optMap, "clocal", "DSR/DTR flow control", CheckYesNo);
  addOption(app, optMap, "crtscts", "RTS/CTS flow control", CheckYesNo);
  addOption(app, optMap, "ixon", "Software Flow Control (XON)", CheckYesNo);
  addOption(app, optMap, "ixoff", "Software Flow Control (XOFF)", CheckYesNo);
  addOption(app, optMap, "break_duration", "Serial Break duration (s)", CheckNumber);
  addOption(app, optMap, "break_delay", "Serial Break delay (s)", CheckNumber);
  addOption(app, optMap, "eventmask", "Serial Event Mask (hex) - see WIN32 SetCommMask(). Specify 0x1FF to enable all events", CheckNumber);
  addOption(app, optMap, "flush", "Flush after write", CheckYesNo);
  addOption(app, optMap, "purge", "Discard characters in input/output buffer (See WIN32 PurgeComm())", CheckPurge);
  addOption(app, optMap, "escape", "Change serial line status (See WIN32 EscapeCommFunction())", CheckEscape);
  
  CLI11_PARSE(app, argc, argv);
  
  const int IOBUF_SIZE = 10000, EOS_SIZE = 10;
  char input[IOBUF_SIZE], input_escaped[IOBUF_SIZE], output[IOBUF_SIZE], inputEos[EOS_SIZE], outputEos[EOS_SIZE];
  size_t nIn = 0, nOut = 0;
  int eomReason = 0, n;
  asynStatus readStatus, writeStatus;

  if (serial_port.compare(0, 3, "COM") == 0) {
      drvAsynSerialPortConfigure("L0", serial_port.c_str(), 0, 0, 0);
  } else {
      drvAsynIPPortConfigure("L0", serial_port.c_str(), 0, 0, 0);
  }
  
  std::auto_ptr<asynOctetClient> client(new asynOctetClient("L0", 0, nullptr, timeout));
  std::auto_ptr<asynOptionClient> optClient(new asynOptionClient("L0", 0, nullptr));

  for(std::map<std::string, std::string>::const_iterator it = optMap.begin(); it != optMap.end(); ++it)
  {
      if (it->second.size() > 0)
      {
          optClient->setOption(it->first.c_str(), it->second.c_str());
      }
  }
  
  if (output_eos.size() > 0) {
      n = epicsStrnRawFromEscaped(outputEos, sizeof(outputEos), output_eos.c_str(), output_eos.size());
      client->setOutputEos(outputEos, n);
  }
  if (input_eos.size() > 0) {
      n = epicsStrnRawFromEscaped(inputEos, sizeof(inputEos), input_eos.c_str(), input_eos.size());
      client->setInputEos(inputEos, n);
  }

  if (output_string.size() > 0)
  {
      n = epicsStrnRawFromEscaped(output, sizeof(output), output_string.c_str(), output_string.size());
      if (serial_report >= 0)
      {
          printf("\n--- Report prior to write ---\n\n");
          pasynManager->report(stdout, serial_report, "L0");
      } 
      writeStatus = client->write(output, n, &nOut);
  }
  
  if (!noread)
  {
      if (serial_report >= 0)
      {
          printf("\n--- Report prior to read ---\n\n");
          pasynManager->report(stdout, serial_report, "L0");
      }
      input[0] = input_escaped[0] = '\0';
      if (wait > 0.0)
      {
          epicsThreadSleep(wait);
      }
      readStatus = client->read(input, sizeof(input), &nIn, &eomReason);
      if (nIn > 0)
      {
          epicsStrnEscapedFromRaw(input_escaped, sizeof(input_escaped), input, nIn);
      }
  }

  if (serial_report >= 0)
  {
      printf("\n--- Report at end ---\n\n");
      pasynManager->report(stdout, serial_report, "L0");
  }
  
  if (output_string.size() > 0)
  {
      printf("Sent %d bytes (excluding any terminator), status=%s, output:\n%s\n", (int)nOut, pasynManager->strStatus(writeStatus), output_string.c_str());
  }
  if (!noread)
  {
      printf("Received %d bytes (excluding any terminator), status=%s, eomReason=%d, response:\n%s\n", (int)nIn, pasynManager->strStatus(readStatus), eomReason, input_escaped);
  }
  return 0;
}
