#ifndef BENCHMARKER_H
#define BENCHMARKER_H

#include <vector>
#include <string>

// Taken from https://github.com/AutonomyLab/obzerver

namespace cftld_ros
{
namespace util
{

/* Singleton */
class StepBenchmarker {
private:
    long int last_tick;
    typedef std::pair<std::string, double> tick_t;
    std::vector<tick_t> items;
    double update();

    StepBenchmarker();
    // Not implementing these two prevents changing the instance
    StepBenchmarker(const StepBenchmarker& rhs);
    const StepBenchmarker& operator=(const StepBenchmarker& rhs);
public:
    static StepBenchmarker& GetInstance();
    void reset();
    void tick();
    void tick(const std::string& text);
    std::string getstr(const bool clear_screen = false) const;
    void dump(const bool clear_screen = false) const;
};

}  // namesapce util
}  // namespace cftld_ros

#define TICK(STR) (cftld_ros::util::StepBenchmarker::GetInstance().tick(STR))
#endif
