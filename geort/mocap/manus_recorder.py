# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

import numpy as np
import zmq
import threading
import time
import argparse
import geort # Assumes geort is installed

# -----------------------------------------------------------
# 1. ZMQ CLIENT
# -----------------------------------------------------------
class ManusZMQClient:
    def __init__(self, port=8765):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(f"tcp://localhost:{port}")
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket = socket
        self._latest_data = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print(f"ManusZMQClient connected to tcp://localhost:{port}")

    def _recv_loop(self):
        while self._running:
            try:
                if self.socket.poll(timeout=1, flags=zmq.POLLIN):
                    msg = self.socket.recv(flags=zmq.NOBLOCK)
                    arr = np.frombuffer(msg, dtype=np.float32).reshape(21, 3)
                    with self._lock:
                        self._latest_data = arr
            except zmq.Again:
                pass
            except Exception as e:
                print(f"ZMQ Client Error: {e}")
                
    def get_latest_data(self):
        with self._lock:
            return self._latest_data.copy() if self._latest_data is not None else None

    def close(self):
        self._running = False
        self._thread.join()
        self.socket.close()

# -----------------------------------------------------------
# 2. RECORDER CLASS (NO OPENCV WINDOWS)
# -----------------------------------------------------------
class ManusRecorderMocap:
    def __init__(self, port=8765):
        self.client = ManusZMQClient(port=port)
        self.status = 'recording' 
        
    def get(self):
        keypoints = self.client.get_latest_data()
        # Return status without any cv2 visualization
        return {'status': self.status, "result": keypoints}

    def close(self):
        self.client.close()

# -----------------------------------------------------------
# 3. MAIN EXECUTION
# -----------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='right_v1', type=str)
    parser.add_argument('--port', default=8765, type=int)
    args = parser.parse_args()

    mocap = ManusRecorderMocap(port=args.port)
    
    all_results = []
    print(f"\n--- RECORDING STARTED (HEADLESS) ---")
    print(f"Press CTRL+C to Stop and Save to '{args.name}'")

    try:
        start_time = time.time()
        while True:
            result = mocap.get()
            if result["result"] is not None:
                all_results.append(result["result"])
                
                # Print progress every 100 frames so you know it's alive
                if len(all_results) % 100 == 0:
                    print(f"Captured {len(all_results)} frames...", end='\r')
            
            time.sleep(0.001) # Prevent CPU spam

    except KeyboardInterrupt:
        print("\n--- STOPPING ---")
    finally:
        mocap.close()
        
    # Save Data
    if len(all_results) > 0:
        final_data = np.array(all_results)
        save_path = geort.save_human_data(final_data, args.name)
        print(f"✅ Saved {final_data.shape[0]} frames to {save_path}")
    else:
        print("🛑 No data collected.")