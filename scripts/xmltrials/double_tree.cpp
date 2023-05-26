#include "../../include/bt_tests/header.h"
#include "../../include/bt_tests/types_template.h"

// using namespace std::chrono_literals;
using namespace BT;


// We want to use this custom type


class FindBall : public SyncActionNode
{
public:
    explicit FindBall(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name,config)
  {
  }

  static PortsList providedPorts()
  {
    return {InputPort<std::string>("ball_location")};
  }

  NodeStatus tick() override
  {
    // std::this_thread::sleep_for(3s);
    auto res = getInput<std::string>("ball_location");
    // std::cout << "Ball Found" << std::endl;
    // std::vector<double> from_string = convertFromString<std::vector<double>>(ballLocation);
    Pose3D target = convertFromString<Pose3D>(res.value());
    printf("Target positions: [ %.1f, %.1f , %.1f, %.1f, %.1f, %.1f,%.1f]\n", target.x, target.y, target.z, target.ox, target.oy, target.oz, target.ow );
    return NodeStatus::SUCCESS;}
};

// NodeStatus ballClose(TreeNode &self)
// {
//   auto msg = self.getInput<std::string>("ball_location");

//   if (!msg)
//   {
//     throw RuntimeError("missing required input[message]: ", msg.error());
//   }

//   for (const auto position_coordinate : msg.value())
//   {
//     std::cout << position_coordinate << ' ';
//   }
//   std::cout << "That's far away" << std::endl;

//   return NodeStatus::FAILURE;
// }

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<FindBall>("FindBall");

//   PortsList say_something_ports = {InputPort<std::string>("ball_location")};
//   factory.registerSimpleCondition("BallClose", ballClose,say_something_ports);

static const char* xml_text = R"(
    <root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Fallback name="root_fallback">
            <FindBall name="find_ball" ball_location="1.0;2.0;2.1;2;0.0;.0;1.0"/>
        </Fallback>
    </BehaviorTree>
    </root>
    )";

  // Create Tree
  // Position2D pos = {4.0,3.6};
  // std::cout<<pos.x << " " << pos.y<<std::endl;

  // std::string postring = "4.1;5.1";
  // std::cout<<postring<<std::endl;

  // Position2D target = convertFromString<Position2D>(postring);
  // std::cout<<target.x << " " << target.y<<std::endl;


  auto tree = factory.createTreeFromText(xml_text);
    PublisherZMQ publisher_zmq(tree);
  // execute the tree
  tree.tickRoot();

  return 0;
}