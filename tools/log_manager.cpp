/**
 * A simple tool to handle logs received from trackers from the HTC Vive
 */

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
    msg_manager.loadMessages(input_path);
  }

  if (port_read >= 0) {
    // TODO: wait for ctrl-c to stop client and write data
    sleep(5);
  }
  
  // TODO: write data (to '.bin' or '.csv')
  // TODO: optional, select a specific time_slot
}
