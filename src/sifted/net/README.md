# net

UDP ingest (sensor data), WebSocket publisher (operator clients), and the degraded-network simulator (loss, delay, reorder, link drop). Network threads only parse-and-enqueue; they never touch the Engine. Milestone 2.
