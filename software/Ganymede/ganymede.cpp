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

// Declare a local daisy_petal for hardware access
DaisyPetal hw;
Parameter param1, param2, param3, param4, param5, param6;

GanyMode currentMode = GanyMode::Bypass;
TimedSequenceCreator sequenceCreator;
TimedSequence playbackSequence;

bool            pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int             switch1[2], switch2[2], switch3[2], dip[4];

Led led1, led2;

void updateSwitch1() // left=, center=, right=
{
    if (pswitch1[0] == true) {  // left

    } else if (pswitch1[1] == true) {  // right

    } else {   // center

    }      
}

void updateSwitch2() // left=, center=, right=
{
    if (pswitch2[0] == true) {  // left

    } else if (pswitch2[1] == true) {  // right


    } else {   // center

    }    
}


void updateSwitch3() // left=, center=, right=
{
    if (pswitch3[0] == true) {  // left

    } else if (pswitch3[1] == true) {  // right


    } else {   // center

    }    
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

    // 3-way Switch 1
    bool changed1 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch1[i]].Pressed() != pswitch1[i]) {
            pswitch1[i] = hw.switches[switch1[i]].Pressed();
            changed1 = true;
        }
    }
    if (changed1) 
        updateSwitch1();
    

    // 3-way Switch 2
    bool changed2 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch2[i]].Pressed() != pswitch2[i]) {
            pswitch2[i] = hw.switches[switch2[i]].Pressed();
            changed2 = true;
        }
    }
    if (changed2) 
        updateSwitch2();

    // 3-way Switch 3
    bool changed3 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch3[i]].Pressed() != pswitch3[i]) {
            pswitch3[i] = hw.switches[switch3[i]].Pressed();
            changed3 = true;
        }
    }
    if (changed3) 
        updateSwitch3();

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

    // TODO: param4 adjusts depth of modulation (i.e. does volume drop to 0%, 80%, or somewhere in between?)
    // TODO: param5 adjusts attack of modulation (how sharply does volume drop? does it hit full modulation immediately after an event? )
    // TODO: param6 adjusts release of modulation (how quickly does volume return to normal?)
    // use daisysp::AdEnv to implement this.
    // Something like:
    // daisysp::AdEnv modulationEnvelope;
    // modulationEnvelope.Init(hw.AudioSampleRate());
    // modulationEnvelope.SetTime(ADENV_SEG_ATTACK, vparam5);
    // modulationEnvelope.SetTime(ADENV_SEG_DECAY, vparam6);
    // modulationEnvelope.SetMax(1.0f);
    // modulationEnvelope.SetMin(0.0f);
    // modulationEnvelope.SetCurve(0);

    // TODO: switch 1 should switch between different filters (volume, LPF, HPF, Moog Ladder?).
    // TODO: switch 2 should switch between ducking source:
    //       midi note on-offs w/ attack and decay, 
    //       sequence, 
    //       or a simple tap tempo.
    // TODO: switch 3 should switch between ducking mode: normal or inverse

    // TODO: Remove these once we have actual functionality
    (void)vparam1;
    (void)vparam2;
    (void)vparam3;
    (void)vparam4;
    (void)vparam5;
    (void)vparam6;

    // Update timed sequence if in playback mode
    bool shouldDuck = false;
    if (currentMode == GanyMode::Playback && playbackSequence.HasStarted()) {
        UpdateResult result = playbackSequence.Update();
        
        // Duck if a sequence event is (or was) within 100ms
        // Duck if event just fired (within 100ms ago) or is about to fire (within 100ms)
        // Note: millisUntilNextFire == 0 means we're at an event right now, so we should duck
        if (result.millisSinceLastFired <= 100 || result.millisUntilNextFire <= 100) {
            shouldDuck = true;
        }
    }

    for(size_t i = 0; i < size; i++)
    {
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

            // Duck audio to zero if sequence event is within 100ms
            float duckLevel = shouldDuck ? 0.0f : 1.0f;

            // Process your signal here
            // Final mix
            if (pdip[0] == false) {// Mono
                out[0][i] = inL * duckLevel;
                out[1][i] = inL * duckLevel;

            } else { // Stereo
                out[0][i] = inL * duckLevel;
                out[1][i] = inR * duckLevel;
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

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    while(1)
    {
        // Do Stuff Infinitely Here
        System::Delay(10);
    }
}
