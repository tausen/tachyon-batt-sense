// Try to find sensible thresholds given short burst (tested with 3-shot burst).
// Data assumed to be an n-sample array of samples after the initial spike.
bool learn(int16_t ptr, int16_t *data, uint16_t n, profile_t *profile);
