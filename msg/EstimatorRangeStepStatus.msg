# Experimental range-height terrain step detector diagnostics.
uint64 timestamp
uint64 timestamp_sample
uint8 estimator_instance

uint8 EVENT_NONE = 0
uint8 EVENT_STARTED = 1
uint8 EVENT_CONFIRMED = 2
uint8 EVENT_RETURNED = 3
uint8 EVENT_UNSTABLE = 4
uint8 EVENT_TIMEOUT = 5
uint8 EVENT_INTERRUPTED = 6
uint8 EVENT_ENDPOINT_CHANGED = 7
uint8 EVENT_REACQUIRING = 8
uint8 EVENT_REANCHORED = 9
uint8 STATE_TRACKING = 0
uint8 STATE_TRANSITION = 1
uint8 STATE_UNRESOLVED = 2
uint8 STATE_REACQUIRING = 3
uint8 state
uint8 event                         # most recent transition (retained between samples)
uint32 event_count
uint64 event_timestamp

float32 sample_interval            # seconds between range observations
float32 start_threshold            # most recently evaluated candidate-start gate, meters
float32 confirmation_threshold     # instantaneous large-step gate, meters
float32 observation                # tilt-corrected range, meters
float32 prediction                 # IMU-propagated pre-edge range, meters
float32 candidate                  # IMU-propagated candidate endpoint, meters
float32 terrain                    # estimated terrain position, positive down, meters
float32 terrain_delta              # most recent confirmed terrain reset, meters
float32 transition_age             # seconds since the original surface change
float32 motion_uncertainty         # accumulated vertical-motion standard deviation, meters
bool degraded                     # bounded height correction; flow withheld until a surface settles
bool pending
bool stable_baseline
