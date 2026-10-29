#pragma once

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <ros/ros.h>

struct ResourceSample
{
    double rss_mb = 0;
    double hwm_mb = 0;
    double peak_mb = 0;
    double vmsize_mb = 0;
    double proc_swap_mb = 0;
    double mem_total_mb = 0;
    double mem_avail_mb = 0;
    double swap_used_mb = 0;
    double load1 = 0;
    double proc_cpu_pct = 0;
};

inline double kb_to_mb(double kb) { return kb / 1024.0; }

inline ResourceSample sample_resources()
{
    ResourceSample s;
    std::ifstream status("/proc/self/status");
    std::string key;
    long val = 0;
    std::string unit;
    while (status >> key)
    {
        if (key == "VmRSS:") { status >> val >> unit; s.rss_mb = kb_to_mb(val); }
        else if (key == "VmHWM:") { status >> val >> unit; s.hwm_mb = kb_to_mb(val); }
        else if (key == "VmPeak:") { status >> val >> unit; s.peak_mb = kb_to_mb(val); }
        else if (key == "VmSize:") { status >> val >> unit; s.vmsize_mb = kb_to_mb(val); }
        else if (key == "VmSwap:") { status >> val >> unit; s.proc_swap_mb = kb_to_mb(val); }
        else status.ignore(1024, '\n');
    }

    std::ifstream meminfo("/proc/meminfo");
    long mem_total = 0, mem_avail = 0, swap_total = 0, swap_free = 0;
    while (meminfo >> key)
    {
        if (key == "MemTotal:") { meminfo >> mem_total >> unit; }
        else if (key == "MemAvailable:") { meminfo >> mem_avail >> unit; }
        else if (key == "SwapTotal:") { meminfo >> swap_total >> unit; }
        else if (key == "SwapFree:") { meminfo >> swap_free >> unit; }
        else meminfo.ignore(1024, '\n');
    }
    s.mem_total_mb = kb_to_mb(mem_total);
    s.mem_avail_mb = kb_to_mb(mem_avail);
    s.swap_used_mb = kb_to_mb(swap_total - swap_free);

    std::ifstream loadavg("/proc/loadavg");
    loadavg >> s.load1;

    static unsigned long long last_jiffies = 0;
    static double last_wall = 0;
    std::ifstream pstat("/proc/self/stat");
    unsigned long utime = 0, stime = 0;
    if (pstat)
    {
        std::string line;
        std::getline(pstat, line);
        const auto rparen = line.rfind(')');
        if (rparen != std::string::npos)
        {
            unsigned long skip = 0;
            char state = 0;
            std::istringstream rest(line.substr(rparen + 1));
            rest >> state;
            for (int i = 0; i < 10; ++i)
                rest >> skip;
            rest >> utime >> stime;
        }
    }
    const double now = ros::WallTime::now().toSec();
    const unsigned long long jiffies = static_cast<unsigned long long>(utime) + stime;
    const long clk = sysconf(_SC_CLK_TCK);
    if (last_wall > 0 && now > last_wall && clk > 0)
        s.proc_cpu_pct = 100.0 * static_cast<double>(jiffies - last_jiffies) / clk / (now - last_wall);
    last_jiffies = jiffies;
    last_wall = now;
    return s;
}

inline std::string format_resource_line(const ResourceSample &s)
{
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "rss=%.0fMB hwm=%.0fMB peak=%.0fMB vmsize=%.0fMB proc_swap=%.0fMB "
                  "mem_avail=%.0f/%.0fMB swap_used=%.0fMB load1=%.2f cpu=%.0f%%",
                  s.rss_mb, s.hwm_mb, s.peak_mb, s.vmsize_mb, s.proc_swap_mb,
                  s.mem_avail_mb, s.mem_total_mb, s.swap_used_mb, s.load1, s.proc_cpu_pct);
    return std::string(buf);
}
