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
    Recording,
    Playback
};

enum class FilterMode {
    Volume,  // Direct volume multiplication
    LPF,     // Low-pass filter using LadderFilter
    HPF,     // High-pass filter using LadderFilter
};

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

enum class SwitchPosition {
    Left,
    Center,
    Right
};
SwitchPosition getSwitchPosition(bool pswitch[2]) {
    if (pswitch[0] == true) {
        return SwitchPosition::Left;
    } else if (pswitch[1] == true) {
        return SwitchPosition::Right;
    } else {
        return SwitchPosition::Center;
    }
}

void updateSwitch1() // left=Volume, center=HPF, right=LPF
{
    SwitchPosition position = getSwitchPosition(pswitch1);
    switch (position) {
        case SwitchPosition::Left:
            filterMode = FilterMode::Volume;
            break;
        case SwitchPosition::Center:
            filterMode = FilterMode::HPF;
            // Set filter mode to high-pass when switching to HPF
            filterL.SetFilterMode(LadderFilter::FilterMode::HP24);
            filterR.SetFilterMode(LadderFilter::FilterMode::HP24);
            break;
        case SwitchPosition::Right:
            filterMode = FilterMode::LPF;
            // Set filter mode to low-pass when switching to LPF
            filterL.SetFilterMode(LadderFilter::FilterMode::LP24);
            filterR.SetFilterMode(LadderFilter::FilterMode::LP24);
            break;
    }
}

void updateSwitch2() // left=normal, center=normal, right=inverse
{
    inverseDuckingMode = (getSwitchPosition(pswitch2) == SwitchPosition::Right);
}

void updateSwitch3() // left=, center=, right=
{
    auto position = getSwitchPosition(pswitch3);
    (void)position;
}


void UpdateButtons()
{
    static uint32_t recordingStartTime = 0;
    static bool leftFootswitchWasPressed = false;
    
    bool leftFootswitchPressed = hw.switches[Funbox::FOOTSWITCH_1].Pressed();
    bool rightFootswitchPressed = hw.switches[Funbox::FOOTSWITCH_2].Pressed();
    
    // Left footswitch: start/stop recording
    if (leftFootswitchPressed && !leftFootswitchWasPressed) {
        // Just pressed: start recording
        if (currentMode == GanyMode::Bypass || currentMode == GanyMode::Playback) {
            currentMode = GanyMode::Recording;
            recordingStartTime = millis();
            sequenceCreator.StartRecording();
            led1.Set(1.0f);  // LED1 indicates recording
            led2.Set(0.0f);
        }
    } else if (!leftFootswitchPressed && leftFootswitchWasPressed) {
        // Just released: stop recording
        if (currentMode == GanyMode::Recording) {
            playbackSequence = sequenceCreator.StopRecording();
            uint32_t recordingDuration = millis() - recordingStartTime;
            recordingStartTime = 0; // Reset
            
            // Check if sequence duration is >= 1 second (1000ms)
            if (recordingDuration >= 1000) {
                // Sequence is valid - start playback
                currentMode = GanyMode::Playback;
                playbackSequence.Start(true); // Start in looping mode
                led1.Set(0.5f);  // Dim LED1 to indicate playback
            } else {
                // Sequence too short - discard and return to bypass
                playbackSequence.Reset();
                currentMode = GanyMode::Bypass;
                led1.Set(0.0f);
            }
        }
    }
    leftFootswitchWasPressed = leftFootswitchPressed;
    
    // Right footswitch: add sequence point (recording) or cancel playback
    static bool rightFootswitchWasPressed = false;
    static uint32_t led2FlashTime = 0;
    if (!rightFootswitchPressed && rightFootswitchWasPressed) {
        // Just released (tapped)
        if (currentMode == GanyMode::Recording && recordingStartTime > 0) {
            // Add a sequence point at current recording time
            uint32_t elapsedFromStart = millis() - recordingStartTime;
            sequenceCreator.AddSequencePoint(elapsedFromStart);
            // Flash LED2 to indicate point added
            led2.Set(1.0f);
            led2FlashTime = millis();
        } else if (currentMode == GanyMode::Playback) {
            // Cancel playback and return to bypass
            playbackSequence.Reset();
            currentMode = GanyMode::Bypass;
            led1.Set(0.0f);
            led2.Set(0.0f);
            led2FlashTime = 0;
        }
    }
    rightFootswitchWasPressed = rightFootswitchPressed;
    
    // Turn off LED2 flash after 100ms
    if (led2FlashTime > 0 && millis() - led2FlashTime > 100) {
        led2.Set(0.0f);
        led2FlashTime = 0;
    }

    led1.Update();
    led2.Update();
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
    //hw.ProcessAllControls();
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    UpdateButtons();
    UpdateSwitches();

    float vparam1 = param1.Process();
    float vparam2 = param2.Process(); 
    float vparam3 = param3.Process();

    float vparam4 = param4.Process();
    float vparam5 = param5.Process();
    float vparam6 = param6.Process();

    // TODO: param1 should trim the start of the sequence
    // TODO: param2 adjusts speed
    // TODO: param3 trims the end of the sequence

    // param4 adjusts depth of modulation (0-1: 0 = no ducking, 1 = full ducking)
    // param5 adjusts attack time (0-1 mapped to 0.001-0.1 seconds)
    // param6 adjusts release/decay time (0-1 mapped to 0.001-0.5 seconds)
    
    // Map parameters to envelope settings
    float depth = vparam4;  // 0-1 range
    float attackTime = 0.001f + vparam5 * 0.099f;  // 0.001 to 0.1 seconds
    float decayTime = 0.001f + vparam6 * 0.499f;   // 0.001 to 0.5 seconds
    
    // Configure envelope
    // Envelope represents ducking amount: 0 = no ducking, depth = full ducking
    // Volume will be 1 - envelope
    // Envelope internally: attack goes from retrig_val_ to 1.0, decay goes from 1.0 to 0.0
    // We want: attack goes from 0 to depth, decay goes from depth to 0
    // So: max_ = depth (when internal=1.0), min_ = 0.0 (when internal=0.0)
    modulationEnvelope.SetMax(depth);  // Full ducking (envelope = depth when internal=1.0)
    modulationEnvelope.SetMin(0.0f);   // No ducking (envelope = 0.0 when internal=0.0)
    modulationEnvelope.SetTime(ADENV_SEG_ATTACK, attackTime);
    modulationEnvelope.SetTime(ADENV_SEG_DECAY, decayTime);
    modulationEnvelope.SetCurve(0);

    // TODO: switch 3 (or should this be on a dip switch?)should switch between ducking source:
    //       midi note on-offs w/ attack and decay, 
    //       sequence, 
    //       or a simple tap tempo.

    // TODO: Remove these once we have actual functionality
    (void)vparam1;
    (void)vparam2;
    (void)vparam3;

    // Update timed sequence if in playback mode and trigger envelope on events
    if (currentMode == GanyMode::Playback && playbackSequence.HasStarted()) {
        UpdateResult result = playbackSequence.Update();
        
        // Trigger envelope when sequence event fires
        if (result.anyFired) {
            modulationEnvelope.Trigger();
        }
    }

    for(size_t i = 0; i < size; i++)
    {
        float signalEnvelope = modulationEnvelope.Process();
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
            if (pdip[0] == false) {// Mono
                out[0][i] = processedL;
                out[1][i] = processedL;
            } else { // Stereo
                out[0][i] = processedL;
                out[1][i] = processedR;
            }
        }
    }
}
          

int main(void)
{
    hw.Init();

    hw.SetAudioBlockSize(4); 

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

    // Init the LEDs and set activate bypass
    led1.Init(hw.seed.GetPin(Funbox::LED_1),false);
    led1.Update();

    led2.Init(hw.seed.GetPin(Funbox::LED_2),false);
    led2.Update();

    // Initialize modulation envelope
    modulationEnvelope.Init(hw.AudioSampleRate());
    modulationEnvelope.SetMax(0.0f);  // Will be set by parameters in audio callback
    modulationEnvelope.SetMin(0.0f);  // No ducking (envelope = 0.0)
    modulationEnvelope.SetTime(ADENV_SEG_ATTACK, 0.01f);
    modulationEnvelope.SetTime(ADENV_SEG_DECAY, 0.1f);
    modulationEnvelope.SetCurve(0);

    // Initialize filters
    filterL.Init(hw.AudioSampleRate());
    filterL.SetFilterMode(LadderFilter::FilterMode::LP24);
    filterL.SetFreq(5000.0f);  // Default cutoff frequency
    filterL.SetRes(0.2f);      // Low resonance
    
    filterR.Init(hw.AudioSampleRate());
    filterR.SetFilterMode(LadderFilter::FilterMode::LP24);
    filterR.SetFreq(5000.0f);  // Default cutoff frequency
    filterR.SetRes(0.2f);      // Low resonance

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    while(1)
    {
        // Do Stuff Infinitely Here
        System::Delay(10);
    }
}
