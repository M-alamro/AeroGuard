# AeroGuard Dashboard

Live map and RF communications dashboard for the AeroGuard drone system.

## Files

- `rf_dashboard.py`: Dashboard server, live map, and RF console.
- `friend_or_foe.py`: Drone identity verification and zone monitoring.

## Requirements

- Python 3
- Gazebo (`gz`) for live simulation integration
- The Gazebo topics and drone models expected by the scripts

The dashboard uses Python standard-library modules. Additional ROS 2 or Gazebo setup may be required for live operation.

## Run

Start the dashboard:

```bash
python3 rf_dashboard.py
```

Run in demo mode without Gazebo:

```bash
python3 rf_dashboard.py --demo
```

Run the friend-or-foe self-test:

```bash
python3 friend_or_foe.py --selftest
```

## Notes

Keep `rf_dashboard.py` and `friend_or_foe.py` in the same directory. The dashboard looks for the friend-or-foe module next to its own script.
