#include "acd/arc_star_detector.h"


namespace acd  { // acd : Asynchronous Corner Detector

ArcStarDetector::ArcStarDetector() :
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}} {

    // Initialize Surface of Active Events to 0-timestamp
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
}

ArcStarDetector::~ArcStarDetector() {
}


bool ArcStarDetector::isCorner(double et, int ex, int ey, bool ep) {
    // Update Surface of Active Events
    const int pol = ep ? 1 : 0;
    const int pol_inv = (!ep) ? 1 : 0;
    double & t_last = sae_latest_[pol](ex,ey);
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);

    // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
    // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
      t_last = et;
      sae_[pol](ex, ey) = et;
    } else {
      t_last = et;
      return false;
    }

    // Return if too close to the border
    const int kBorderLimit = 4;
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;
    }

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 3;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 4;
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;
    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {
      const double t =sae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);
      if (t > segment_new_min_t) {
        segment_new_min_t = t;
        arc_right_idx = i; // % End up in the maximum value
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {
    is_arc_valid = false;

      segment_new_min_t = sae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =sae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}

} // acd : Asynchronous Corner Detector
