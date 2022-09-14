from mido       import MidiFile
from struct     import pack
import os

def midi_to_bytes(midi_file):
    os.chdir(os.path.dirname(__file__))

    mid   = MidiFile(midi_file, clip=True)
    song  = b""
    track = mid.play()

    for msg in track:
        if msg.time != 0: 
            try: 
                # time != 0 on note_off messages
                msg = next(track)
            except StopIteration: break

            if hasattr(msg, "note"):
                # bytes 1-2 for time, byte 3 for note value
                song += pack("<HB", round(msg.time * 1000), msg.note)

                if len(song) % 50 * 3 == 0: 
                    print(f"written {len(song)} bytes")

    return song

if __name__ == "__main__":
    from pyperclip  import copy

    print("Converting MIDI...")

    song = midi_to_bytes("MIDI/megalovania.mid")
    copy(str(song))

    print(f"Finished ({len(song)} bytes)")
    print("Data copied to clipboard.")
