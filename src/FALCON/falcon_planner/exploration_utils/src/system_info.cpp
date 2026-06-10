#include "system_info.h"

namespace {
bool parseLong(const std::string &value, long &result) {
  try {
    result = std::stol(value);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
} // namespace

void printSystemInfo(std::string &output) {
  std::stringstream ss;
  ss << "|---------------------------------- System Info ----------------------------------|"
            << std::endl;
  std::string line;
  std::string cpu_name, cpu_cores, cpu_threads, cpu_freq;
  std::ifstream cpuinfo("/proc/cpuinfo");
  if (cpuinfo.is_open()) {
    while (getline(cpuinfo, line)) {
      if (line.find("model name") != std::string::npos) {
        cpu_name = line.substr(line.find(":") + 2);
      } else if (line.find("cpu cores") != std::string::npos) {
        cpu_cores = line.substr(line.find(":") + 2);
      } else if (line.find("siblings") != std::string::npos) {
        cpu_threads = line.substr(line.find(":") + 2);
      } else if (line.find("cpu MHz") != std::string::npos) {
        cpu_freq = line.substr(line.find(":") + 2);
      }
    }
    cpuinfo.close();
  } else {
    std::cerr << "Unable to open /proc/cpuinfo" << std::endl;
  }

  ss << "CPU Name: " << cpu_name << std::endl;
  ss << "CPU Cores: " << cpu_cores << std::endl;
  ss << "CPU Threads: " << cpu_threads << std::endl;
  ss << "CPU Frequency: " << cpu_freq << " MHz" << std::endl;

  std::ifstream meminfo("/proc/meminfo");
  std::string mem_total, mem_free;
  if (meminfo.is_open()) {
    while (getline(meminfo, line)) {
      if (line.find("MemTotal") != std::string::npos) {
        mem_total = line.substr(line.find(":") + 2);
      } else if (line.find("MemFree") != std::string::npos) {
        mem_free = line.substr(line.find(":") + 2);
      }
    }
    meminfo.close();
  } else {
    std::cerr << "Unable to open /proc/meminfo" << std::endl;
  }

  long mem_total_kb, mem_free_kb;
  if (parseLong(mem_total, mem_total_kb))
    ss << "Memory Total: " << mem_total_kb / 1024.0 / 1024.0 << " GB" << std::endl;
  else
    ss << "Memory Total: unavailable" << std::endl;

  if (parseLong(mem_free, mem_free_kb))
    ss << "Memory Free: " << mem_free_kb / 1024.0 / 1024.0 << " GB" << std::endl;
  else
    ss << "Memory Free: unavailable" << std::endl;

  // gpu info
  std::string gpu_name, gpu_mem_total, gpu_mem_free;
  std::string command =
      "nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader 2>/dev/null";
  FILE *fp = popen(command.c_str(), "r");
  if (fp == NULL) {
    std::cerr << "Failed to run command nvidia-smi" << std::endl;
  } else {
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), fp) != NULL) {
      std::string line(buffer);
      size_t pos = line.find(",");
      if (pos == std::string::npos)
        continue;
      gpu_name = line.substr(0, pos);
      line = line.substr(pos + 1);
      pos = line.find(",");
      if (pos == std::string::npos)
        continue;
      gpu_mem_total = line.substr(0, pos);
      gpu_mem_free = line.substr(pos + 1);
    }
    pclose(fp);
  }

  long gpu_mem_total_mb, gpu_mem_free_mb;
  ss << "GPU Name: " << (gpu_name.empty() ? "unavailable" : gpu_name) << std::endl;
  if (parseLong(gpu_mem_total, gpu_mem_total_mb))
    ss << "GPU Memory Total: " << gpu_mem_total_mb / 1024.0 << " GB" << std::endl;
  else
    ss << "GPU Memory Total: unavailable" << std::endl;

  if (parseLong(gpu_mem_free, gpu_mem_free_mb))
    ss << "GPU Memory Free: " << gpu_mem_free_mb / 1024.0 << " GB" << std::endl;
  else
    ss << "GPU Memory Free: unavailable" << std::endl;

  ss << "|---------------------------------------------------------------------------------|"
            << std::endl;
    
  output = ss.str();
}
