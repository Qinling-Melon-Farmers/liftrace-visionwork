// Read-only client for an ALREADY RUNNING Gazebo Classic world.
// It starts no server, spawns no model and sends only the world_sdf request.
#include <iostream>
#include <string>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

int main(int argc, char **argv)
{
  if (argc != 2 || std::string(argv[1]).empty())
  {
    std::cerr << "Usage: read_runtime_world_sdf WORLD_NAME\n";
    return 2;
  }
  const std::string world(argv[1]);
  if (!gazebo::client::setup())
    return 3;
  const auto response = gazebo::transport::request(
      world, "world_sdf", "", gazebo::common::Time(10, 0));
  gazebo::msgs::GzString data;
  const bool ok = response && response->response() == "success" &&
      response->type() == data.GetTypeName() &&
      data.ParseFromString(response->serialized_data()) && !data.data().empty();
  if (ok)
    std::cout << data.data();
  else
    std::cerr << "No successful world_sdf response; runtime model is unverified\n";
  gazebo::client::shutdown();
  return ok ? 0 : 4;
}
