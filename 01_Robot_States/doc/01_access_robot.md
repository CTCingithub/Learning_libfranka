# ���ӻ�����

## 1. �ٷ�ʾ������

�������Ӻû����ˣ�������`FCI`�󣬿�ͨ���ٷ�ʾ������`print_joint_poses`�õ������˵�ǰ���ؽ�λ�ˡ�

```console
$ print_joint_poses 172.16.0.2
[0.996195,0.0871569,0,0,-0.0871569,0.996195,0,0,0,0,1,0,0,0,0.333,1]
[0.732421,0.0640794,0.67783,0,0.675251,0.0590776,-0.735219,0,-0.0871569,0.996195,0,0,0,0,0.333,1]
[0.729513,0.0936028,0.677532,0,-0.108845,0.993855,-0.0201075,0,-0.675251,-0.0590776,0.735219,0,-0.213514,-0.0186803,0.565476,1]
[0.102232,-0.00892871,-0.994721,0,0.988788,0.110326,0.100632,0,0.108845,-0.993855,0.0201075,0,-0.153329,-0.0109581,0.621373,1]
[0.101835,-0.005316,-0.994787,0,-0.109216,0.993881,-0.0164915,0,0.988788,0.110326,0.100632,0,0.218129,0.0321659,0.7421,1]
[0.993671,0.109601,0.0245929,0,-0.0262499,0.0137012,0.999562,0,0.109216,-0.993881,0.0164915,0,0.218129,0.0321659,0.7421,1]
[0.719616,-0.69379,0.0284081,0,-0.693875,-0.720047,-0.00835229,0,0.0262499,-0.0137012,-0.999562,0,0.305572,0.0418108,0.744264,1]
[0.719616,-0.69379,0.0284081,0,-0.693875,-0.720047,-0.00835229,0,0.0262499,-0.0137012,-0.999562,0,0.308381,0.0403448,0.637311,1]
[0.99949,0.0185659,0.0259935,0,0.0182016,-0.999734,0.0141816,0,0.0262499,-0.0137012,-0.999562,0,0.311095,0.0389281,0.533956,1]
```

Դ����`print_joint_poses.cpp`���ⲿ�����£�

```c++
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    franka::RobotState state = robot.readOnce();

    franka::Model model(robot.loadModel());
    for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
         frame++) {
      std::cout << model.pose(frame, state) << std::endl;
    }
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
```

## 2. ��Ҫ�޸�

ʹ��`yaml-cpp`�⣬��������IP��ַ���������ļ��У��Ӷ�ʹ��Ŀ���淶��

```c++
std::string file = std::filesystem::current_path().parent_path().string() + "/config/FR3.yaml";
    YAML::Node config = YAML::LoadFile(file);
    const std::string robot_ip = config["Robot_ip"].as<std::string>();
    std::cout << "IP Address of Franka Research 3 Robot:" << robot_ip << std::endl;
```

�����ȡһ�ιؽڽǶȣ���������նˡ�

```c++
franka::Robot Myrobot(robot_ip);
franka::RobotState current_state = Myrobot.readOnce();

// Print Read Joint Angle(rad)
std::cout << "Read Joint Angle(rad): "
          << current_state.q << std::endl;

// Print Read Joint Angle(degree)
std::cout << "Read Joint Angle(degree): [";
for (const auto &joint : current_state.q)
{
  std::cout << joint / M_PI * 180;
  if (&joint != &current_state.q.back())
  {
    std::cout << ", ";
  }
}
std::cout << "]" << std::endl;
```

���н����

```console
$ ./01_access_robot 
IP Address of Franka Research 3 Robot: 172.16.0.2
Read Joint Angle(rad): [0.0872911, -0.744719, 0.0296455, -2.21447, 0.00365254, 1.49453, 0.876626]
Read Joint Angle(degree): [5.00141, -42.6693, 1.69856, -126.88, 0.209275, 85.6305, 50.227]
```
