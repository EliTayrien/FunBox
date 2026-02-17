#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "asym_clipper.h"
#include "mc_common.h"

using namespace daisy;
using namespace daisysp;
using namespace funbox;
using namespace SpaceFeelingsSoftware::MicroControllers;

// Declare a local daisy_petal for hardware access
DaisyPetal hw;
// overdrive and wavefolder's process function is idempotent, so we can use one instance for both channels
Overdrive overdrive;
Wavefolder wavefolder;
// asym clipper is not idempotent - it includes antialiasing filters that "remember" the last sample processed, 
// so we need to use one instance for each channel
AsymClipper asymClipperL;
AsymClipper asymClipperR;

Parameter gainParam, driveParam, volumeParam, hpfParam, lpfParam, asymParam;

bool bypass;

bool pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int switch1[2], switch2[2], switch3[2], dip[4];

Led led1, led2;

enum class Routing {
    Mono,
    Stereo,
    MISO,
};

Routing getRouting() {
    // pdip 0 indicates mono or stereo in
    // pdip 1 indicates mono or stereo out
    bool stereoIn = pdip[0];
    bool stereoOut = pdip[1];

    if (stereoIn) {
        // stereo in, mono out is not supported.
        // if dip switch 1 (stereo in) is set, we can assume we're in full stereo mode.
        return Routing::Stereo;
    } else if (stereoOut) {
        return Routing::MISO;
    } else {
        return Routing::Mono;
    }
}

Routing routing;
bool overdriveEnabled = false;
bool wavefolderEnabled = false;
bool asymClipperEnabled = false;

void updateSwitch1() {
    // TODO: soft clipping, hard clipping, asymmetric clipping
    auto position = Funbox::GetSwitchPosition(pswitch1);
    overdriveEnabled = position == Funbox::SwitchPosition::Right;
}

void updateSwitch2() {
    auto position = Funbox::GetSwitchPosition(pswitch2);
    wavefolderEnabled = position == Funbox::SwitchPosition::Right;
}

void updateSwitch3() {
    auto position = Funbox::GetSwitchPosition(pswitch3);
    asymClipperEnabled = position == Funbox::SwitchPosition::Right;
}

void UpdateButtons() {
    // (De-)Activate bypass and toggle LED when left footswitch is let go, or enable/disable amp if held for greater than 1 second //
    // Can only disable/enable amp when not in bypass mode
    if(hw.switches[Funbox::FOOTSWITCH_1].FallingEdge())
    {
        bypass = !bypass;
        led1.Set(bypass ? 0.0f : 1.0f);
    }

    led1.Update();
    led2.Update();
}

void UpdateSwitches() {
    // Detect any changes in switch positions (3 On-Off-On switches and Dip switches)

    // 3-way Switch 1
    bool changed1 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch1[i]].Pressed() != pswitch1[i]) {
            pswitch1[i] = hw.switches[switch1[i]].Pressed();
            changed1 = true;
        }
    }
    if (changed1) {
        updateSwitch1();
    }
    

    // 3-way Switch 2
    bool changed2 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch2[i]].Pressed() != pswitch2[i]) {
            pswitch2[i] = hw.switches[switch2[i]].Pressed();
            changed2 = true;
        }
    }
    if (changed2) {
        updateSwitch2();
    }

    // 3-way Switch 3
    bool changed3 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch3[i]].Pressed() != pswitch3[i]) {
            pswitch3[i] = hw.switches[switch3[i]].Pressed();
            changed3 = true;
        }
    }
    if (changed3) {
        updateSwitch3();
    }

    // Dip switches
    for(int i=0; i<2; i++) {
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
    hw.ProcessAllControls();

    UpdateButtons();
    UpdateSwitches();

    float gain = gainParam.Process();
    float drive = driveParam.Process();
    float volume = volumeParam.Process();
    float hpf = hpfParam.Process();
    float lpf = lpfParam.Process();
    float asym = asymParam.Process();

    overdrive.SetDrive(drive);
    wavefolder.SetGain(2 * drive);
    wavefolder.SetOffset(SpaceFeelingsSoftware::MicroControllers::lerp(asym, 0.0f, 3.0f));

    asymClipperL.SetHPF(hpf);
    asymClipperL.SetLPF(lpf);
    asymClipperL.SetQ(SpaceFeelingsSoftware::MicroControllers::lerp(asym, AsymClipper::QMin, AsymClipper::QMax));
    asymClipperR.SetHPF(hpf);
    asymClipperR.SetLPF(lpf);
    asymClipperR.SetQ(SpaceFeelingsSoftware::MicroControllers::lerp(asym, AsymClipper::QMin, AsymClipper::QMax));

    // Handle Knob Changes Here
    for(size_t i = 0; i < size; i++)
    {
        // Process your signal here
        if(bypass) {
            out[0][i] = in[0][i]; 
            out[1][i] = in[1][i];
        }
        else {
            float inL = in[0][i] * gain;
            float inR = in[1][i] * gain;

            auto wetL = inL;
            auto wetR = inR;

            if (overdriveEnabled) {
                wetL = overdrive.Process(wetL);
                wetR = overdrive.Process(wetR);
            }
            if (wavefolderEnabled) {
                wetL = wavefolder.Process(wetL);
                wetR = wavefolder.Process(wetR);
            }
            if (asymClipperEnabled) {
                wetL = asymClipperL.Process(wetL);
                wetR = asymClipperR.Process(wetR);
            }

            if (getRouting() == Routing::Mono) {
                // ignore Right channel
                out[0][i] = wetL * volume;
            }
            else if (getRouting() == Routing::Stereo) {
                // use both channels
                out[0][i] = wetL * volume;
                out[1][i] = wetR * volume;
            }
            else if (getRouting() == Routing::MISO) {
                // use left channel
                out[0][i] = wetL * volume;
                out[1][i] = wetL * volume;
            }
        }
    }
}
          

int main(void)
{
    hw.Init();

    overdrive.Init();
    overdrive.SetDrive(0.5);
    wavefolder.Init();
    wavefolder.SetOffset(0.5);
    wavefolder.SetGain(0.5);

    asymClipperL.Init(hw.AudioSampleRate());
    asymClipperR.Init(hw.AudioSampleRate());

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


    gainParam.Init(hw.knob[Funbox::KNOB_1], 1.0f, 3.0f, Parameter::LINEAR);
    driveParam.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    volumeParam.Init(hw.knob[Funbox::KNOB_3], 0.0f, 2.0f, Parameter::LINEAR); 
    hpfParam.Init(hw.knob[Funbox::KNOB_4], AsymClipper::HPFMin, AsymClipper::HPFMax, Parameter::LINEAR);
    lpfParam.Init(hw.knob[Funbox::KNOB_5], AsymClipper::LPFMin, AsymClipper::LPFMax, Parameter::LINEAR);
    asymParam.Init(hw.knob[Funbox::KNOB_6], 0, 1, Parameter::LINEAR);

    // Init the LEDs and set activate bypass
    led1.Init(hw.seed.GetPin(Funbox::LED_1),false);
    led1.Update();
    bypass = true;

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