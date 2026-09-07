# Etat d'ensemble du robot, publie par le gestionnaire de modes (2 Hz,
# transient_local) : une seule source pour l'interface, les scripts et les
# journaux. Les metriques materielles fines restent dans /diagnostics_agg ;
# ici ne figure que ce qui decide du comportement.

std_msgs/Header header

# Mode courant (constantes ci-dessous) et sous-etat libre, lisible par un
# humain (« en transition vers LOCALIZATION », « but en cours », ...).
uint8 MODE_IDLE=0
uint8 MODE_TELEOP=1
uint8 MODE_MAPPING=2
uint8 MODE_LOCALIZATION=3
uint8 MODE_EXPLORATION=4
uint8 MODE_DOCKING=5
uint8 mode
string mode_name
string substate
bool transitioning

# Localisation : quelle source publie map -> odom, et sa qualite.
string localization_source      # "", "slam_toolbox", "amcl"
bool localized                  # AMCL converge / SLAM a traite des scans
float32 localization_quality    # 0..1, derive de la covariance AMCL (1 = sur)
string map_name                 # carte chargee ou en cours, "" sinon

# Navigation nav2.
bool nav_active                 # lifecycle manager : noeuds actifs
bool goal_active
string goal_status              # "", "accepted", "succeeded", "aborted", "canceled"
float32 distance_remaining      # m, 0 sans but

# Sante, resumee depuis /diagnostics_agg (niveaux diagnostic_msgs).
uint8 health                    # 0 OK, 1 WARN, 2 ERROR, 3 STALE
string[] errors                 # libelles des groupes en WARN ou plus
bool lidar_ok
bool camera_ok
bool imu_ok
bool pico_ok
