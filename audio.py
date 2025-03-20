from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from typing import Tuple
import sys

def json2audiovec(filename: str) -> Tuple[AudioNoteVector, float]:
    with open(filename) as json_input:
        json_data = eval(json_input.read())
        note_list = json_data['records']
        final_time = note_list[-1][0]
        midi_times = [(n[1][1], n[0]) for n in note_list if n[1][0] == 144]
        audio_msg = AudioNoteVector()
        audio_msg.append = False
        audio_msg.notes = []
        for i in range(len(midi_times)):
            next_time = midi_times[i + 1][1] if i + 1 < len(midi_times) else final_time
            duration = next_time - midi_times[i][1]
            note = AudioNote()
            note.frequency = int(440 * 2**((midi_times[i][0] - 69) / 12))
            note.max_runtime = Duration(sec=int(duration), nanosec=(int((duration % 1) * 10**9)))
            audio_msg.notes.append(note)
        return (audio_msg, final_time)
    
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} filename")
    else:
        for filename in sys.argv[1:]:
            print(json2audiovec(filename))
