
#include "acd/arc_star_detector.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;

struct Event {
    double t;
    int x,y;
    bool p;
};

int LoadEventFile (const std::string file_path,
                    std::vector<Event>& event_vec,
                    const size_t n_event = 30e6,
                    const size_t expected_num_event = 30e6) {
  std::ifstream file(file_path);

  if(file.fail()) {
   std::cout << "Error opening file" << std::endl <<std::flush;
   return -1;
  }

  event_vec.clear(); // Event vec must be empty
  event_vec.reserve(expected_num_event);  // Preallocate vectors


  size_t i_event = 0;
  while (!file.eof() &&
         (n_event==0 || i_event < n_event)) { // maximum number of events reached
    ++i_event;
    double t;
    int x,y;
    bool p;

    file >> t >> x >> y >> p; // Input format from the Event Camera Dataset. Adapt to your file format

    Event event = {t,x,y,p};
    event_vec.push_back(event);
  }
  file.close();

  event_vec.shrink_to_fit(); // Free memory if possible
  return 0;
}

template<class T>
int WriteContainerToFile (const std::string& file_path, const T& container){
  std::ofstream file (file_path);

  if (file.fail()) {
      std::cout << "Error creating/opening file" << std::endl <<std::flush;
      return -1;
  }

  for (auto item : container) {
    file << item << std::endl;
  }
  file.close();
}

using namespace acd; // Asynchronous Corner Detector

int main(int argc, char *argv[]) {

    ArcStarDetector detector = ArcStarDetector();

  // Input files
  if (!(argc==2 || argc==3)) {
      std::cerr << "Usage: " << argv[0] << "  my_dataset_folder/events.txt [my_result_folder/classification.txt]" << std::endl;
      return 1;
  }
  std::string event_file_path = argv[1];

  // Load events
  std::vector<Event> event_vec;
  std::cout << "Loading event file..." << std::flush;
  LoadEventFile(event_file_path, event_vec, 0); // Load all the events (n_event=0). Note: if you expect to load a massive amount of events consider pre-reserving larger amount
  std::cout << "OK" << std::endl;

  if (event_vec.size()==0) {
    std::cout << "Event file contains no events... exiting"<< std::endl;
    return -1;
  }

  // Output classification vector (true (1) - event-corner and false (0) otherwise)
  std::vector<bool> corner_vec(event_vec.size(),false);
  size_t corner_count = 0;

  std::cout << "Processing events..."  << std::flush;
  auto t_init = Clock::now();
  for (size_t i = 0, i_end = event_vec.size(); i<i_end;++i) {
      const Event& e = event_vec[i];
      // Detect Corner
      if (detector.isCorner(e.t, e.x, e.y, e.p)) {
        corner_vec[i] = true;
        corner_count++;
      }
  }
  auto t_final= Clock::now();
  std::cout << "OK" << std::endl;
  std::cout << std::endl;

  // Summary stats
  const double elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t_final - t_init).count();
  const double percentage_corners = (corner_count/double(event_vec.size()))*100;
  const double time_per_event = elapsed_time/event_vec.size(); // Average time to process one event [ns/ev]
  const double event_rate = 1/(time_per_event*1e-3) ; // Average Event Rate [Million ev / s]
  const double real_time_factor = event_vec[event_vec.size()-1].t/(elapsed_time*1e-9);

  std::cout << "Corner classification stats: " << std::endl
            << "\t Num. Events: " << event_vec.size() << std::endl
            << "\t Num. Corners: " << corner_count << std::endl
            << "\t Reduction Percentage: "  << percentage_corners << "%" << std::endl << std::endl
            << "Timing stats: " << std::endl
            << " \tAvg. Time per event: " << time_per_event << " [ns/ev]" << std::endl
            << " \tAvg. Max. Event Rate: " << event_rate <<  " [Mev/s]" << std::endl
            << " \tAvg. Real-time factor: " << real_time_factor << "x" <<std::endl<<std::endl;


  // Output corner classification file
  if (argc==3) {
      std::string corner_file_path = argv[2];
      std::cout << "Writing corner classification to file..." << std::flush;
      WriteContainerToFile(corner_file_path,corner_vec);
      std::cout << "OK" << std::endl;
  }
}



