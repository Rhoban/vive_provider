/**
 * A simple tool to handle logs received from trackers from the HTC Vive
 */

#include <vive_provider/udp_message_manager.h>

#include <tclap/CmdLine.h>

using namespace vive_provider;

int main(int argc, char ** argv) {
  TCLAP::CmdLine cmd("Handle logs received from trackers from the HTC Vive",
                     ' ', "1.0");
  TCLAP::ValueArg<int> port_arg("p", "port",
                                "The port on which vive messages are received (-1: no listener)",
                                false, -1, "port", cmd);
  TCLAP::ValueArg<std::string> input_arg("i", "input",
                                         "An optional path to existing logs",
                                          false, "", "path", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output",
                                          "The path to which the logs will be saved",
                                          true, "data_vive.bin", "path", cmd);
  try {
    cmd.parse(argc, argv);
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    exit(EXIT_FAILURE);
  }

  int port_read = port_arg.getValue();
  std::string input_path  = input_arg.getValue();
  std::string output_path = output_arg.getValue();

  UDPMessageManager msg_manager(port_read, -1);

  if (input_path != "") {
    std::cout << "loading messages" << std::endl;
    msg_manager.loadMessages(input_path);
    std::cout << "Nb messages loaded: " << msg_manager.getMessages().size() << std::endl;
  }

  if (input_path == "" && port_read == -1) {
    std::cerr << "No input_path nor port_read provided, cannot acquire data" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (port_read >= 0) {
    std::cout << "Reading messages: press q to save and quit" << std::endl;
    char key = ' ';
    while (key != 'q') {
      std::cin >> key;
    }
  }
  std::cout << "Saving " << msg_manager.getMessages().size() << " messages" << std::endl;

  msg_manager.saveMessages(output_path);
  
  // TODO: create a '.csv' option
  // TODO: optional, select a specific time_slot
}
