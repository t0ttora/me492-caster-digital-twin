// Follow a time-tagged Gazebo fixture trajectory, independent of robot drive commands.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

namespace janus::geometry
{
class MoveFixture final : public gz::sim::System, public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
  private: struct Waypoint
  {
    double seconds;
    double lateral;
  };

  private: gz::sim::Entity model = gz::sim::kNullEntity;
  private: gz::sim::Entity world = gz::sim::kNullEntity;
  private: gz::math::Pose3d initial;
  private: std::vector<Waypoint> waypoints;
  private: std::atomic<double> origin{-1.0};
  private: double lastLateral = 0.0;
  private: gz::transport::Node transport;
  private: gz::transport::Node::Publisher position;
  private: std::ofstream log;

  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    world = entity;
    for (auto step = sdf->FindElement("waypoint"); step; step = step->GetNextElement("waypoint"))
      waypoints.push_back({step->Get<double>("time_s"), step->Get<double>("y_m")});
    if (waypoints.empty() || !std::is_sorted(waypoints.begin(), waypoints.end(),
        [](const Waypoint &a, const Waypoint &b) {return a.seconds < b.seconds;}))
      throw std::runtime_error("Invalid JANUS crossing fixture time sequence");

    log.open(sdf->Get<std::string>("output"));
    if (!log)
      throw std::runtime_error("Could not open JANUS crossing fixture trace");
    log << "sim_time_ns,x_m,y_m,z_m,commanded_y_m,origin_sim_time_s\n"
        << std::setprecision(12);
    transport.Subscribe("/geometry/scene_start", &MoveFixture::Arm, this);
    position = transport.Advertise<gz::msgs::Pose>("/geometry/fixture_pose");
  }

  private: void Arm(const gz::msgs::Double &message)
  {
    double unset = -1.0;
    if (std::isfinite(message.data()) && message.data() >= 0.0)
      origin.compare_exchange_strong(unset, message.data());
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused)
      return;
    if (model == gz::sim::kNullEntity)
    {
      model = gz::sim::World(world).ModelByName(ecm, "crossing_fixture");
      if (model == gz::sim::kNullEntity)
        return;
      initial = gz::sim::worldPose(model, ecm);
      lastLateral = initial.Pos().Y();
    }
    const auto start = origin.load();
    if (start < 0.0)
      return;
    const double elapsed = std::chrono::duration<double>(info.simTime).count() - start;
    auto first = waypoints.begin();
    if (elapsed < first->seconds)
      return;
    auto next = std::upper_bound(first, waypoints.end(), elapsed,
        [](double time, const Waypoint &waypoint) {return time < waypoint.seconds;});
    const auto &last = *(next-1);
    double y = last.lateral;
    if (next != waypoints.end())
    {
      const double progress = (elapsed-last.seconds)/(next->seconds-last.seconds);
      y += progress*(next->lateral-last.lateral);
    }
    if (y != lastLateral)
    {
      auto target = initial;
      target.Pos().Y(y);
      gz::sim::Model(model).SetWorldPoseCmd(ecm, target);
      lastLateral = y;
    }
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || model == gz::sim::kNullEntity || info.iterations % 20 != 0)
      return;
    const auto pose = gz::sim::worldPose(model, ecm);
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        info.simTime).count();
    const auto &actual = pose.Pos();
    log << timestamp << ',' << actual.X() << ',' << actual.Y() << ',' << actual.Z()
        << ',' << lastLateral << ',' << origin.load() << '\n';
    if (info.iterations % 100 == 0)
      log.flush();
    gz::msgs::Pose message;
    message.mutable_header()->mutable_stamp()->set_sec(timestamp/1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(timestamp%1000000000);
    message.set_name("crossing_fixture");
    message.mutable_position()->set_x(actual.X());
    message.mutable_position()->set_y(actual.Y());
    message.mutable_position()->set_z(actual.Z());
    message.mutable_orientation()->set_w(1.0);
    position.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(janus::geometry::MoveFixture, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
