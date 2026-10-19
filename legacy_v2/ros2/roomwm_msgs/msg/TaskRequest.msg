# A high-level task only. Receipt always disarms; this message cannot issue ARM.
uint16 schema_version
string<=128 goal_id
string<=16 kind
string<=500 text
string<=16 direction
