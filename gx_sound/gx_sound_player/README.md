# gx_sound_player

## Install dependencies

```sh
$ sudo apt install vorbis-tools
```

## Usage

```sh
$ roslaunch gx_sound_player sound_player.launch
```


## Action API

### Action Subscribed Topics

- `~sound_request/goal` (gx_sound_msgs/SoundRequestActionGoal)
    - A goal with audio file path and stamp for sound player to play.
- `~sound_request/cancel` (actionlib_msgs/GoalID)
    - A request to cancel a specific goal.

### Action Published Topics

- `~sound_request/feedback` (gx_sound_msgs/SoundRequestActionFeedback)
    - No detailed information is contained in feedback.
- `~sound_request/result` (gx_sound_msgs/SoundRequestActionResult)
    - Status information of the goals (succeed / interrupted / file not found).
- `~sound_request/status` (actionlib_msgs/GoalStatusArray)


#### Parameters

- `~device_name` (str)
    - Name of audio device to play sound (ex: "default", "plughw:1,0")
