# Complete Guide: Connecting Python to STM32CubeMonitor

## What You Need

1. **STM32CubeMonitor** installed (download from ST website)
2. **Python tuning interface** running
3. Your robot connected and sending data

## Part 1: Start the Python Server

### Step 1: Run Your Tuning Interface
```bash
cd c:\Users\dalya\Desktop\NxP_Cup\src
python tuning_interface.py
```

### Step 2: Enable CubeMonitor Server
In the Python window:
1. Find the checkbox that says **"Enable STM32CubeMonitor"**
2. Click it to enable
3. You should see: **"Server: ON (Port 8000)"** in green text
4. Leave this window open!

### Step 3: Connect to Your Robot
1. Enter ESP32 IP address (default: 192.168.4.1)
2. Click **"Connect"** button
3. Send some velocity commands to make sure data is flowing

---

## Part 2: Setup STM32CubeMonitor

### Step 1: Open STM32CubeMonitor
1. Launch **STM32CubeMonitor** application
2. You'll see a node-based flow editor (like Node-RED)

### Step 2: Create New Flow
1. Click the **"+"** button at the top to create a new flow tab
2. Name it "Robot Data" or whatever you prefer

### Step 3: Add HTTP Request Node
1. On the left sidebar, find the **"network"** section
2. Drag the **"http request"** node onto the canvas
3. Double-click the node to configure it:
   - **Method**: Select `GET`
   - **URL**: Type `http://localhost:8000/data`
   - **Return**: Select `a parsed JSON object`
   - **Name**: Type "Get Robot Data"
   - Click **"Done"**

### Step 4: Add Inject Node (Timer)
1. Find **"inject"** node in the left sidebar (under "common")
2. Drag it onto the canvas
3. Double-click to configure:
   - **Repeat**: Select `interval`
   - **Every**: Type `0.1` (for 100 milliseconds)
   - **Name**: Type "Poll Every 100ms"
   - Click **"Done"**

### Step 5: Add Debug Node
1. Find **"debug"** node in the left sidebar (under "common")
2. Drag it onto the canvas
3. Double-click to configure:
   - **Output**: Select `complete msg object`
   - **To**: Select `debug window`
   - Click **"Done"**

### Step 6: Connect the Nodes
1. Click and drag from the **inject node's output** (right side) to the **http request node's input** (left side)
2. Click and drag from the **http request node's output** to the **debug node's input**
3. You should see lines connecting them: `[inject] → [http request] → [debug]`

### Step 7: Deploy and Test
1. Click the **"Deploy"** button (top right, red button)
2. Open the **Debug** panel (click bug icon on right sidebar)
3. Click the button on the left side of the **inject node** (small square button)
4. In the debug panel, you should see JSON data appear:
   ```json
   {
     "timestamp": 1234567890.123,
     "robot_distance_mm": 1500.5,
     "orientation_deg": 45.2,
     "left_velocity_mm_s": 250.0,
     ...
   }
   ```

---

## Part 3: Add Charts and Gauges

### For Line Charts (Distance, Velocity, etc.)

1. Find **"chart"** node in left sidebar (under "dashboard")
2. Drag it onto canvas
3. Double-click to configure:
   - **Group**: Create new group called "Velocity Data"
   - **Type**: Select `Line chart`
   - **X-axis**: Type `10` (shows last 10 seconds)
   - **Label**: Type "Left/Right Velocity"
   - Click **"Done"**

4. Add a **"function"** node before the chart:
   ```javascript
   // Extract velocity data
   msg.payload = [{
       "series": ["Left", "Right"],
       "data": [[msg.payload.left_velocity_mm_s], 
                [msg.payload.right_velocity_mm_s]],
       "labels": [""]
   }];
   return msg;
   ```

5. Connect: `[http request] → [function] → [chart]`

### For Gauges (Current Values)

1. Find **"gauge"** node in sidebar
2. Drag onto canvas
3. Configure:
   - **Type**: Select `Gauge`
   - **Range**: min `0`, max `1000` (for velocity in mm/s)
   - **Label**: "Left Velocity"
   - Click **"Done"**

4. Add **"change"** node to extract just one value:
   - **Set**: `msg.payload`
   - **To**: `msg.payload.left_velocity_mm_s`

5. Connect: `[http request] → [change] → [gauge]`

### Repeat for Other Variables
Create similar flows for:
- Distance (gauge + line chart)
- Orientation (gauge + line chart)
- Servo angle (gauge)
- Cross-track error (line chart)

---

## Part 4: View the Dashboard

1. Click the **"dashboard"** icon on the right sidebar (looks like a grid)
2. Click the **"open dashboard"** button (arrow icon)
3. A browser window opens showing your live graphs and gauges!
4. Your robot data updates in real-time!

---

## Complete Example Flow

Here's what your final flow should look like:

```
[inject: 100ms] → [http request: localhost:8000] → [debug]
                                                  ↓
                                            [function: parse left vel] → [gauge: Left Velocity]
                                                  ↓
                                            [function: parse right vel] → [gauge: Right Velocity]
                                                  ↓
                                            [function: parse distance] → [chart: Distance]
                                                  ↓
                                            [function: parse orientation] → [chart: Heading]
```

---

## Troubleshooting

### "Connection refused" error in CubeMonitor
- **Problem**: Python server not running
- **Solution**: Make sure checkbox "Enable STM32CubeMonitor" is checked and shows green status

### "No data" in debug panel
- **Problem**: URL might be wrong
- **Solution**: 
  1. Open browser and go to `http://localhost:8000/data`
  2. You should see JSON data
  3. Copy this exact URL to CubeMonitor http request node

### Data shows but doesn't update
- **Problem**: Inject node not repeating
- **Solution**: 
  1. Double-click inject node
  2. Make sure "Repeat" is set to "interval"
  3. Set to 0.1 seconds (100ms)
  4. Click "Done" and "Deploy"

### Python says "Address already in use"
- **Problem**: Port 8000 is being used by another program
- **Solution**: 
  1. Close any other applications using port 8000
  2. Or change port in Python code to 8001, then use `http://localhost:8001/data` in CubeMonitor

---

## Quick Test Without CubeMonitor

To verify your Python server is working:

1. Enable STM32CubeMonitor in Python interface
2. Open your web browser
3. Go to: `http://localhost:8000/data`
4. You should see JSON data
5. Refresh the page - numbers should update

If this works, then CubeMonitor will work too!

---

## Available Data Fields

| Field Name | Description | Unit |
|------------|-------------|------|
| `timestamp` | Time data was captured | seconds |
| `robot_distance_mm` | Total distance traveled | millimeters |
| `orientation_deg` | Robot heading angle | degrees |
| `left_velocity_mm_s` | Left wheel speed | mm/s |
| `right_velocity_mm_s` | Right wheel speed | mm/s |
| `servo_angle_deg` | Steering angle | degrees |
| `cross_track_error_mm` | Distance from lane center | millimeters |

---

## Video Tutorial Links

STMicroelectronics has official tutorials:
- STM32CubeMonitor Getting Started: https://www.st.com/en/development-tools/stm32cubemonitor.html
- Node-RED basics (CubeMonitor uses Node-RED): https://nodered.org/docs/tutorials/

---

## Need Help?

1. **Check Python console** - any errors will show there
2. **Check CubeMonitor debug panel** - shows exactly what data is received
3. **Test with browser** - `http://localhost:8000/data` should work
4. **Make sure robot is sending data** - Python needs to receive data from ESP32 first!
