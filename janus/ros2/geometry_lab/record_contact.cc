// Read the solver's existing contact components once per simulation step.
// The transport contact publisher omits empty samples.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>

namespace janus::geometry
{
class RecordContact final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  private: struct Channel
  {
    std::string collision;
    gz::sim::Entity id = gz::sim::kNullEntity;
  };

  private: std::array<Channel, 4> channels{{
    {"arche::base_link::chassis"},
    {"arche::left_wheel::tire"},
    {"arche::right_wheel::tire"},
    {"arche::caster_wheel::tire"},
  }};
  private: std::ofstream log;

  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    log.open(sdf->Get<std::string>("output"));
    if (!log)
      throw std::runtime_error("Could not open JANUS native contact trace");

    log << "sim_time_ns,collision_entity,component_ready,contact_pairs,contact_points,"
        << "wrench_points,force_x_N,force_y_N,force_z_N,normal_load_N\n"
        << std::setprecision(12);
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || info.dt.count() <= 0)
      return;

    if (channels.back().id == gz::sim::kNullEntity)
    {
      ecm.Each<gz::sim::components::ContactSensorData>(
          [this, &ecm](const gz::sim::Entity &id, const auto *)
          {
            auto name = gz::sim::scopedName(id, ecm, "::", false);
            for (auto &channel : channels)
            {
              if (name == channel.collision ||
                  (name.size() > channel.collision.size() &&
                   name.compare(name.size()-channel.collision.size(),
                                channel.collision.size(), channel.collision) == 0))
                channel.id = id;
            }
            return true;
          });
    }

    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        info.simTime).count();
    for (const auto &channel : channels)
    {
      const auto *data = ecm.Component<gz::sim::components::ContactSensorData>(channel.id);
      std::array<double, 3> force{};
      int pairs = 0, points = 0, wrenches = 0;
      double normal = 0.0;
      if (data)
      {
        for (const auto &contact : data->Data().contact())
        {
          ++pairs;
          points += contact.position_size();
          const bool ownIsFirst = contact.collision1().id() == channel.id;
          for (int i = 0; i < contact.wrench_size(); ++i)
          {
            const auto &wrench = contact.wrench(i);
            const auto &f = ownIsFirst ? wrench.body_1_wrench().force() :
                wrench.body_2_wrench().force();
            ++wrenches;
            force[0] += f.x();
            force[1] += f.y();
            force[2] += f.z();
            if (i < contact.normal_size())
            {
              const auto &n = contact.normal(i);
              normal += std::abs(f.x()*n.x() + f.y()*n.y() + f.z()*n.z());
            }
          }
        }
      }
      log << timestamp << ',' << channel.collision << ',' << (data != nullptr) << ','
          << pairs << ',' << points << ',' << wrenches << ',' << force[0] << ','
          << force[1] << ',' << force[2] << ',' << normal << '\n';
    }

    if (info.iterations % 100 == 0)
      log.flush();
  }
};
}

GZ_ADD_PLUGIN(janus::geometry::RecordContact, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
