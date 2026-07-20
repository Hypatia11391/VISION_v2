#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <vector>
#include <chrono>
#include <thread>

int main() {
    auto nt = nt::NetworkTableInstance::GetDefault();
    nt.StartClient4("RaspberryPiVision");
    nt.SetServer("192.168.0.44", 5810);

    auto table = nt.GetTable("Vision");
    auto posePub = table->GetDoubleArrayTopic("robot_pose").Publish();

    while (true) {
        std::vector<double> fakePose = {1.5, 2.3, 0.0, 0.0, 0.0, 45.0, 1.0, 2.0, 3.0, 4.0, static_cast<double>(nt::Now())/1000000.0};
        
        posePub.Set(fakePose);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
