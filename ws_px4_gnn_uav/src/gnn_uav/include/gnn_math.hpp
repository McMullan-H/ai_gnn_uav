// Math definitions for calculating various values

#define pi acos(-1)
#define rad(a) a * pi / 180
#define avg(a, b) (a + b) / 2

// Approximate length of one degree latitude in meters w.r.t. latitude
#define LAT_M(lat) ( 111132.92 - 559.82 * cos(2 * rad(lat)) + 1.175 * cos(4 * rad(lat)) - 0.0023 * cos(6 * rad(lat)) )

// Approximate length of one degree longitude in meters w.r.t. latitude
#define LON_M(lat) ( 111412.84 * cos(rad(lat)) - 93.5 * cos(3 * rad(lat)) + 0.118 * cos(5 * rad(lat)) )

// Latitude/longitude conversion macros
#define LAT_TO_M(a, b, lat) (a - b) * LAT_M(lat)
#define LON_TO_M(a, b, lat) (a - b) * LON_M(lat)
#define M_TO_LAT(a, b, lat) (a - b) / LAT_M(lat)
#define M_TO_LON(a, b, lat) (a - b) / LON_M(lat)

// Z-velocity formula as a cosine function w.r.t. the hover height setpoint
#define Z_VEL(current_position, setpoint_position, rise_rate) ( -(cos((current_position / -setpoint_position) * pi / 2) * rise_rate) )