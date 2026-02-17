#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "mc_common.h"
#include "timed_sequence.h"

//
// Ganymede - A pedal using timed sequences to duck audio levels
// Allows for Stereo In/Out, 6 knobs, 3 3-way switches, 4 dipswitches, 2 SPST Footswitches, 2 LEDs.
//
// Modes:
// - bypass: audio is unaffected
// - recording: recording a timed sequence to duck with
// - playback: using recorded sequence to duck audio levels
//

using namespace daisy;
using namespace daisysp;
using namespace funbox;  // This is important for mapping the correct controls to the Daisy Seed on Funbox PCB
using namespace SpaceFeelingsSoftware::MicroControllers;  // mc_common namespace

// Provide millis() compatibility for mc_common
uint32_t millis() {
    return daisy::System::GetNow();
}

enum class GanyMode {
    Bypass,
    RecordingSequence,
    Playback
};

enum class FilterMode {
    Volume,  // Direct volume multiplication
    LPF,     // Low-pass filter using LadderFilter
    HPF,     // High-pass filter using LadderFilter
};

enum class EventSource {
    TapSequence,
    MIDI,
    // TODO: Tap Tempo
};
EventSource eventSource = EventSource::TapSequence;
 
constexpr float MIN_ATTACK_TIME = 0.01f;
constexpr float MAX_ATTACK_TIME = 0.5f;
constexpr float MIN_DECAY_TIME = 0.01f;
constexpr float MAX_DECAY_TIME = 1.5f;

constexpr uint32_t MIN_SEQUENCE_DURATION = 1000;

// Declare a local daisy_petal for hardware access
DaisyPetal hw;
Parameter param1, param2, param3, param4, param5, param6;

GanyMode currentMode = GanyMode::Bypass;
TimedSequenceCreator sequenceCreator;
TimedSequence playbackSequence;
AdEnv modulationEnvelope;

FilterMode filterMode = FilterMode::Volume;
bool inverseDuckingMode = false;
LadderFilter filterL, filterR;

// Helper function to map envelope value (0-1) to frequency (Hz) with exponential curve
// This provides more musical control - lower values have finer resolution
inline float envelopeToFrequency(float envelope, float minFreq = 20.0f, float maxFreq = 20000.0f) {
    // Exponential mapping: envelope^2 gives more control in lower frequencies
    float normalized = envelope * envelope;  // Square for exponential curve
    return minFreq + normalized * (maxFreq - minFreq);
}

bool            pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int             switch1[2], switch2[2], switch3[2], dip[4];

Led led1, led2;

void updateSwitch1() // left=Volume, center=HPF, right=LPF
{
    Funbox::SwitchPosition position = Funbox::GetSwitchPosition(pswitch1);
    switch (position) {
        case Funbox::SwitchPosition::Left:
            filterMode = FilterMode::Volume;
            break;
        case Funbox::SwitchPosition::Center:
            filterMode = FilterMode::HPF;
            // Set filter mode to high-pass when switching to HPF
            filterL.SetFilterMode(LadderFilter::FilterMode::HP24);
            filterR.SetFilterMode(LadderFilter::FilterMode::HP24);
            break;
        case Funbox::SwitchPosition::Right:
            filterMode = FilterMode::LPF;
            // Set filter mode to low-pass when switching to LPF
            filterL.SetFilterMode(LadderFilter::FilterMode::LP24);
            filterR.SetFilterMode(LadderFilter::FilterMode::LP24);
            break;
    }
}

void updateSwitch2() // left=normal, center=normal, right=inverse
{
    inverseDuckingMode = (Funbox::GetSwitchPosition(pswitch2) == Funbox::SwitchPosition::Right);
}

void updateSwitch3() // left=, center=, right=
{
    auto position = Funbox::GetSwitchPosition(pswitch3);
    if (position == Funbox::SwitchPosition::Left) {
        eventSource = EventSource::MIDI;
    } else {
        eventSource = EventSource::TapSequence;
    }
}

uint32_t recordingStartTime = 0;
float signalEnvelope = 0.0f;

void startRecordingSequence() {
    currentMode = GanyMode::RecordingSequence;
    recordingStartTime = millis();
    sequenceCreator.StartRecording();
    led1.Set(1.0f);  // LED1 indicates recording
    led2.Set(0.0f);
    hw.seed.PrintLine("Mode: RecordingSequence");
}

void stopRecordingSequence() {
    // Just released: stop recording
    playbackSequence = sequenceCreator.StopRecording();
    uint32_t recordingDuration = millis() - recordingStartTime;
    recordingStartTime = 0; // Reset
    
    if (recordingDuration >= MIN_SEQUENCE_DURATION) {
        currentMode = GanyMode::Playback;
        playbackSequence.Start(true);
        hw.seed.PrintLine("Mode: Playback");
    } else {
        // Sequence too short - discard and return to bypass
        playbackSequence.Reset();
        currentMode = GanyMode::Bypass;
        hw.seed.PrintLine("Mode: Bypass (sequence too short)");
    }
}

constexpr uint32_t TURN_OFF_LED2_DELAY_MS = 100;
Timer turnOffLED2;

void UpdateButtons()
{   
    if (eventSource == EventSource::MIDI && hw.switches[Funbox::FOOTSWITCH_1].RisingEdge()) {
        currentMode = currentMode == GanyMode::Bypass ? GanyMode::Playback : GanyMode::Bypass;
    }

    if (eventSource == EventSource::TapSequence) {
        // Left footswitch: start/stop recording
        if (hw.switches[Funbox::FOOTSWITCH_1].RisingEdge() && currentMode != GanyMode::RecordingSequence) {
            startRecordingSequence();
        } else if (hw.switches[Funbox::FOOTSWITCH_1].FallingEdge() && currentMode == GanyMode::RecordingSequence) {
            stopRecordingSequence();
        }

        // Right footswitch: add sequence point (recording) or cancel playback
        if (hw.switches[Funbox::FOOTSWITCH_2].FallingEdge()) {
            // Just released (tapped)
            if (currentMode == GanyMode::RecordingSequence && recordingStartTime > 0) {
                // Add a sequence point at current recording time
                uint32_t elapsedFromStart = millis() - recordingStartTime;
                sequenceCreator.AddSequencePoint(elapsedFromStart);
                // Flash LED2 to indicate point added
                turnOffLED2.reset();
                turnOffLED2.start(TURN_OFF_LED2_DELAY_MS);
                hw.seed.PrintLine("Point added");
            } else if (currentMode == GanyMode::Playback) {
                // Cancel playback and return to bypass
                playbackSequence.Reset();
                currentMode = GanyMode::Bypass;
                hw.seed.PrintLine("Mode: Bypass (playback cancelled)");
            }
        }
    }
}


void UpdateSwitches()
{
    // Detect any changes in switch positions (3 On-Off-On switches and Dip switches)
    int* switches[] = {switch1, switch2, switch3};
    bool* pswitches[] = {pswitch1, pswitch2, pswitch3};
    for (int switchIndex = 0; switchIndex < 3; switchIndex++) {
        auto switchArray = switches[switchIndex];
        bool changed = false;
        for (int i = 0; i < 2; i++) {
            if (hw.switches[switchArray[i]].Pressed() != pswitches[switchIndex][i]) {
                pswitches[switchIndex][i] = hw.switches[switchArray[i]].Pressed();
                changed = true;
            }
        }
        if (changed && switchIndex == 0 ) {
            updateSwitch1();
        } else if (changed && switchIndex == 1) {
            updateSwitch2();
        } else if (changed && switchIndex == 2) {
            updateSwitch3();
        }
    }

    // Dip switches
    for(int i=0; i<4; i++) {
        if (hw.switches[dip[i]].Pressed() != pdip[i]) {
            pdip[i] = hw.switches[dip[i]].Pressed();
            // Action for dipswitches handled in audio callback
        }
    }
}

// This runs at a fixed rate, to prepare audio samples
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{

    for(size_t i = 0; i < size; i++)
    {
        signalEnvelope = modulationEnvelope.Process();
        if (!modulationEnvelope.IsRunning()) {
            signalEnvelope = 0.0f;
        }
        if (inverseDuckingMode) {
            signalEnvelope = 1.0f - signalEnvelope;
        }

        // Process your signal here
        if(currentMode != GanyMode::Playback)
        {
            out[0][i] = in[0][i]; 
            out[1][i] = in[1][i];
        }
        else
        {
            float inL = in[0][i];
            float inR = in[1][i];

            // Process signal based on filter mode
            float processedL = inL;
            float processedR = inR;

            if (filterMode == FilterMode::LPF) {
                // Apply low-pass filter with envelope-controlled cutoff
                // signalEnvelope: 0 = low cutoff (more filtering), 1 = high cutoff (less filtering)
                float cutoffFreq = envelopeToFrequency(signalEnvelope);
                filterL.SetFreq(cutoffFreq);
                filterR.SetFreq(cutoffFreq);
                processedL = filterL.Process(inL);
                processedR = filterR.Process(inR);
            } else if (filterMode == FilterMode::HPF) {
                // Apply high-pass filter with envelope-controlled cutoff
                // signalEnvelope: 0 = low cutoff (more filtering), 1 = high cutoff (less filtering)
                float cutoffFreq = envelopeToFrequency(signalEnvelope);
                filterL.SetFreq(cutoffFreq);
                filterR.SetFreq(cutoffFreq);
                processedL = filterL.Process(inL);
                processedR = filterR.Process(inR);
            } else if (filterMode == FilterMode::Volume) {
                // Direct volume multiplication
                processedL = inL * signalEnvelope;
                processedR = inR * signalEnvelope;
            }

            // Final mix
            if (pdip[0] == false) { // Mono
                out[0][i] = processedL;
                out[1][i] = processedL;
            } else { // Stereo
                out[0][i] = processedL;
                out[1][i] = processedR;
            }
        }
    }
}

void UpdateEnvelope(float modulationDepth, float normalizedAttackRate, float normalizedDecayRate) {
    
    modulationEnvelope.SetMax(modulationDepth);
    modulationEnvelope.SetMin(0.0f);
    modulationEnvelope.SetTime(ADENV_SEG_ATTACK, 
        map<float>(normalizedAttackRate, 0.0f, 1.0f, MIN_ATTACK_TIME, MAX_ATTACK_TIME));
    modulationEnvelope.SetTime(ADENV_SEG_DECAY, 
        map<float>(normalizedDecayRate, 0.0f, 1.0f, MIN_DECAY_TIME, MAX_DECAY_TIME));
    modulationEnvelope.SetCurve(0);

    // Process MIDI messages and check for Note On on channel 1
    bool noteOnMessageReceived = false;
    hw.midi.Listen();
    while(hw.midi.HasEvents())
    {
        MidiEvent m = hw.midi.PopEvent();
        // Check for MIDI channel 1 (channel 0 in 0-indexed) and non-zero velocity
        // This is to avoid Max/MSP Note outs for now..
        if(m.type == NoteOn && m.channel == 0 && m.data[1] != 0)
        {
            noteOnMessageReceived = true;
        }
    }

    bool playbackSequenceFired = false;
    if (playbackSequence.HasStarted()) {
        auto result = playbackSequence.Update();
        playbackSequenceFired = result.anyFired;
    }

    bool triggerForSequence = eventSource == EventSource::TapSequence && playbackSequenceFired;
    bool triggerForMidi = eventSource == EventSource::MIDI && noteOnMessageReceived;
    if (currentMode == GanyMode::Playback && (triggerForSequence || triggerForMidi)) {
        modulationEnvelope.Trigger();
    }
}

int main(void)
{
    hw.Init();
    hw.seed.StartLog(false);
    hw.seed.PrintLine("Starting Ganymede");
    
    hw.SetAudioBlockSize(48); 
    float samplerate = hw.AudioSampleRate();

    switch1[0]= Funbox::SWITCH_1_LEFT;
    switch1[1]= Funbox::SWITCH_1_RIGHT;
    switch2[0]= Funbox::SWITCH_2_LEFT;
    switch2[1]= Funbox::SWITCH_2_RIGHT;
    switch3[0]= Funbox::SWITCH_3_LEFT;
    switch3[1]= Funbox::SWITCH_3_RIGHT;
    dip[0]= Funbox::SWITCH_DIP_1;
    dip[1]= Funbox::SWITCH_DIP_2;
    dip[2]= Funbox::SWITCH_DIP_3;
    dip[3]= Funbox::SWITCH_DIP_4;

    pswitch1[0]= false;
    pswitch1[1]= false;
    pswitch2[0]= false;
    pswitch2[1]= false;
    pswitch3[0]= false;
    pswitch3[1]= false;
    pdip[0]= false;
    pdip[1]= false;
    pdip[2]= false;
    pdip[3]= false;

    param1.Init(hw.knob[Funbox::KNOB_1], 0.0f, 1.0f, Parameter::LINEAR);
    param2.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    param3.Init(hw.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR); 
    param4.Init(hw.knob[Funbox::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
    param5.Init(hw.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
    param6.Init(hw.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR); 


    led1.Init(hw.seed.GetPin(Funbox::LED_1),false);
    led1.Set(0.0f);
    led1.Update();

    led2.Init(hw.seed.GetPin(Funbox::LED_2),false);
    led2.Set(0.0f);
    led2.Update();

    // Initialize modulation envelope
    modulationEnvelope.Init(samplerate);
    modulationEnvelope.SetMax(1.0f);
    modulationEnvelope.SetMin(0.0f);
    modulationEnvelope.SetTime(ADENV_SEG_ATTACK, 0.01f);
    modulationEnvelope.SetTime(ADENV_SEG_DECAY, 0.1f);
    modulationEnvelope.SetCurve(0);

    // Initialize filters
    filterL.Init(samplerate);
    filterL.SetFilterMode(LadderFilter::FilterMode::LP24);
    filterL.SetFreq(5000.0f);  // Default cutoff frequency
    filterL.SetRes(0.2f);      // Low resonance
    
    filterR.Init(samplerate);
    filterR.SetFilterMode(LadderFilter::FilterMode::LP24);
    filterR.SetFreq(5000.0f);  // Default cutoff frequency
    filterR.SetRes(0.2f);      // Low resonance

    hw.InitMidi();
    hw.midi.StartReceive();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    
    while(1)
    {
        hw.ProcessAllControls();

        UpdateButtons();
        UpdateSwitches();

        float vparam1 = param1.Process();
        float vparam2 = param2.Process(); 
        float vparam3 = param3.Process();

        // TODO: param1 should trim the start of the sequence
        // TODO: param2 adjusts speed
        // TODO: param3 trims the end of the sequence
        (void)vparam1;
        (void)vparam2;
        (void)vparam3;
        
        UpdateEnvelope(param4.Process(), param5.Process(), param6.Process());

        if (currentMode == GanyMode::Bypass) {
            led1.Set(0.0f);
            led2.Set(0.0f);
        } else if (currentMode == GanyMode::RecordingSequence) {
            led1.Set(1.0f);
            led2.Set(turnOffLED2.isRunning() ? 1.0f : 0.0f);
        } else if (currentMode == GanyMode::Playback) {
            led1.Set(0.5f);
            led2.Set(signalEnvelope);
        }
        led1.Update();
        led2.Update();

        System::Delay(10);
    }
}
