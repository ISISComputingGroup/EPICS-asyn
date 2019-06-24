#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <map>

#include <epicsString.h>
#include <drvAsynSerialPort.h>
#include <asynPortClient.h>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define NO_CLI11_SUPPORT
#endif

#ifndef NO_CLI11_SUPPORT

#include <CLI/CLI.hpp>

static CLI::Validator CheckYesNo = CLI::IsMember({"Y", "N"}, CLI::ignore_case);
static CLI::Validator CheckNumber = CLI::Number;
static CLI::Validator CheckParity = CLI::IsMember({"odd", "even", "none", "mark", "space"});

static void addOption(CLI::App& app, std::map<std::string, std::string>& optMap, const std::string& opt, const std::string& desc, CLI::Validator validator)
{
    app.add_option(std::string("--") + opt, optMap[opt], desc)->check(validator);
}	

// testSerialPort [options] COMPORT [outputString] [outputEos] [inputEos]

int main(int argc, char *argv[])
{
  CLI::App app;
  std::map<std::string, std::string> optMap;

  std::string serial_port;
  std::string output_string;
  std::string output_eos;
  std::string input_eos;
  int serial_report = -1;
  bool noread = false;

  app.add_option("--port,port", serial_port, "COM Port")->mandatory();
  app.add_option("output_string", output_string, "Output String");
  app.add_option("output_eos", output_eos, "Output EOS");
  app.add_option("input_eos", input_eos, "Input EOS");
  app.add_option("--report", serial_report, "Run reports of this level (dbior)");
  app.add_flag("--noread", noread, "Do not read after write");
  
  addOption(app, optMap, "baud", "Baud Rate", CheckNumber);
  addOption(app, optMap, "bits", "Data Bits", CheckNumber);
  addOption(app, optMap, "parity", "Parity", CheckParity);
  addOption(app, optMap, "clocal", "DSR/DTR flow control", CheckYesNo);
  addOption(app, optMap, "crtscts", "RTS/CTS flow control", CheckYesNo);
  addOption(app, optMap, "ixon", "Software Flow Control (XON)", CheckYesNo);
  addOption(app, optMap, "ixoff", "Software Flow Control (XOFF)", CheckYesNo);
  addOption(app, optMap, "break_duration", "Serial Break duration", CheckNumber);
  addOption(app, optMap, "break_delay", "Serial Break delay", CheckNumber);
  addOption(app, optMap, "eventmask", "Serial Event Mask (hex) - see SetCommMask(), 0x1FF enables all", CheckNumber);
  addOption(app, optMap, "flush", "Flush after write", CheckYesNo);
  
  CLI11_PARSE(app, argc, argv);

  char input[10000], inputEos[10], outputEos[10];
  size_t nIn = 0, nOut = 0;
  int eomReason = 0;
  asynStatus readStatus, writeStatus;

  drvAsynSerialPortConfigure("L0", serial_port.c_str(), 0, 0, 0);
  
  std::auto_ptr<asynOctetClient> client(new asynOctetClient("L0", 0, 0));
  std::auto_ptr<asynOptionClient> optClient(new asynOptionClient("L0", 0, 0));

  for(std::map<std::string, std::string>::const_iterator it = optMap.begin(); it != optMap.end(); ++it)
  {
	  if (it->second.size() > 0)
	  {
	      optClient->setOption(it->first.c_str(), it->second.c_str());
	  }
  }
  
  if (output_eos.size() > 0) {
      epicsStrnRawFromEscaped(outputEos, sizeof(outputEos), output_eos.c_str(), output_eos.size());
      client->setOutputEos(outputEos, static_cast<int>(strlen(outputEos)));
  }
  if (input_eos.size() > 0) {
      epicsStrnRawFromEscaped(inputEos, sizeof(inputEos), input_eos.c_str(), input_eos.size());
      client->setInputEos(inputEos, static_cast<int>(strlen(inputEos)));
  }

  if (output_string.size() > 0)
  {
      if (serial_report >= 0)
      {
	      printf("\n--- Report prior to write ---\n\n");
		  pasynManager->report(stdout, serial_report, "L0");
      } 
      writeStatus = client->write(output_string.c_str(), output_string.size(), &nOut);
  }
  
  if (!noread)
  {
      if (serial_report >= 0)
      {
	      printf("\n--- Report prior to read ---\n\n");
		  pasynManager->report(stdout, serial_report, "L0");
      }
	  input[0] = '\0';
      readStatus = client->read(input, sizeof(input), &nIn, &eomReason);
  }

  if (serial_report >= 0)
  {
 	  printf("\n--- Report at end ---\n\n");
      pasynManager->report(stdout, serial_report, "L0");
  }
  
  if (output_string.size() > 0)
  {
      printf("Sent %d bytes, status=%s, output:\n%s\n", (int)nOut, pasynManager->strStatus(writeStatus), output_string.c_str());
  }
  if (!noread)
  {
      printf("Received %d bytes, status=%s, eomReason=%d, response:\n%s\n", (int)nIn, pasynManager->strStatus(readStatus), eomReason, input);
  }
  return 0;
}

#else

// VS2010 version with no options support

// testSerialPort COMPORT [outputString] [outputEos] [inputEos]

int main(int argc, char *argv[])
{
  std::string serial_port;
  std::string output_string;
  std::string output_eos;
  std::string input_eos;
  int serial_report = 5;
  bool noread = false;
  
  if (argc > 1)
  {
      serial_port = argv[1];
  }
  if (argc > 2)
  {
      output_string = argv[2];
  }
  if (argc > 3)
  {
      output_eos = argv[3];
  }
  if (argc > 4)
  {
      input_eos = argv[4];
  }
  
  char input[10000], inputEos[10], outputEos[10];
  size_t nIn = 0, nOut = 0;
  int eomReason = 0;
  asynStatus readStatus, writeStatus;

  drvAsynSerialPortConfigure("L0", serial_port.c_str(), 0, 0, 0);
  
  std::auto_ptr<asynOctetClient> client(new asynOctetClient("L0", 0, 0));
  std::auto_ptr<asynOptionClient> optClient(new asynOptionClient("L0", 0, 0));

  optClient->setOption("eventmask", "0x1ff");
  
  if (output_eos.size() > 0) {
      epicsStrnRawFromEscaped(outputEos, sizeof(outputEos), output_eos.c_str(), output_eos.size());
      client->setOutputEos(outputEos, static_cast<int>(strlen(outputEos)));
  }
  if (input_eos.size() > 0) {
      epicsStrnRawFromEscaped(inputEos, sizeof(inputEos), input_eos.c_str(), input_eos.size());
      client->setInputEos(inputEos, static_cast<int>(strlen(inputEos)));
  }

  if (output_string.size() > 0)
  {
      if (serial_report >= 0)
      {
	      printf("\n--- Report prior to write ---\n\n");
		  pasynManager->report(stdout, serial_report, "L0");
      } 
      writeStatus = client->write(output_string.c_str(), output_string.size(), &nOut);
  }
  
  if (!noread)
  {
      if (serial_report >= 0)
      {
	      printf("\n--- Report prior to read ---\n\n");
		  pasynManager->report(stdout, serial_report, "L0");
      }
	  input[0] = '\0';
      readStatus = client->read(input, sizeof(input), &nIn, &eomReason);
  }

  if (serial_report >= 0)
  {
 	  printf("\n--- Report at end ---\n\n");
      pasynManager->report(stdout, serial_report, "L0");
  }
  
  if (output_string.size() > 0)
  {
      printf("Sent %d bytes, status=%s, output:\n%s\n", (int)nOut, pasynManager->strStatus(writeStatus), output_string.c_str());
  }
  if (!noread)
  {
      printf("Received %d bytes, status=%s, eomReason=%d, response:\n%s\n", (int)nIn, pasynManager->strStatus(readStatus), eomReason, input);
  }
  return 0;
}

#endif /* ifndef NO_CLI11_SUPPORT */
