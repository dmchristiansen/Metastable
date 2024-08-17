#include "plugin.hpp"
#include "dsp/digital.hpp"

const uint8_t nWeights = 20;

struct Weight {
	float mass;
	float position;
	float velocity;
	float acceleration;
};

struct MassSystem {

	Weight weights[nWeights] = {{}};
	float spring[nWeights] = {};
	float center[nWeights] = {};
	float damp[nWeights] = {};

	float hammer[4][nWeights];

	void initHammer() {
		for(int i = 0; i < nWeights; i++) {
			hammer[0][i] = sinf((static_cast<float>(i) / static_cast<float>(nWeights)) * 2 * M_PI);
			hammer[1][i] = (i > nWeights / 2) ? 1.0 : -1.0;
			hammer[2][i] = (static_cast<float>(i) / static_cast<float>(nWeights)) * 2.0 - 1.0;
			hammer[3][i] = sinf(static_cast<float>(i) / static_cast<float>(nWeights) * 2 * M_PI);
			DEBUG("h0 %f", hammer[0][i]);
		}
	}

	void setMass(float mass_) {
		float m = clamp(mass_, 0.1, 5.0);
		for (int i = 0; i < nWeights; i++) {
			weights[i].mass = m;
		}
	}

	void setSpring(float spring_) {
		float s = clamp(spring_, 0.0, 5.0);
		for (int i = 0; i < nWeights; i++) {
			spring[i] = s;
		}	
	}

	void setCenter(float center_) {
		float c = clamp(center_, 0.0, 5.0);
		for (int i = 0; i < nWeights; i++) {
			center[i] = c;
		}
	}

	void setDamp(float damp_) {
		float d = clamp(damp_, 0.0, 5.0);
		for (int i = 0; i < nWeights; i++) {
			damp[i] = d;
		}
	}

	void update(float delta) {
		float h = clamp(delta, 0.001, 0.1);

		// update velocity from acceleration
		for (int i = 0; i < nWeights; i++) {
			weights[i].velocity += weights[i].acceleration * h;
		}
		
		// update position from velocity
		for (int i = 0; i < nWeights; i++) {
			weights[i].position += weights[i].velocity * h;
		}

		// update acceleration
		float leftForce = (weights[nWeights-1].position - weights[0].position) * spring[nWeights-1];
		float rightForce, centerForce, dampForce;
		for (int i = 0; i < nWeights; i++) {
			rightForce = (weights[(i+1)%nWeights].position - weights[i].position) * spring[i];
			centerForce = -1.0 * weights[i].position * center[i];
			dampForce = -1.0 * weights[i].velocity * damp[i];
			weights[i].acceleration = (leftForce + rightForce + centerForce + dampForce) / weights[i].mass;
			leftForce = rightForce * -1.0;
		}
		//for (int i = 0; i < nWeights; i++) {
		//	weights[i].acceleration = (-1.0 * center[i] * weights[i].position) / weights[i].mass;
		//}
	}

	float sample(float phase) {
		float indexMix = phase * static_cast<float>(nWeights);
		double indexInt;
		float indexFrac = modf(indexMix, &indexInt);

		int32_t index = static_cast<int32_t>(indexInt);
		float amplitude = (weights[index].position * (1 - indexFrac)) + (weights[(index + 1) % nWeights].position * indexFrac);
		return amplitude;
	}
	
	void strike(float strength, float shape) {
		
		double indexInt;
		float indexFrac = modf(clamp(shape, 0.0, 3.0), &indexInt);
		float strengthFrac = clamp(strength, 0.0, 1.0);
		int32_t index = static_cast<int32_t>(indexInt);
		for (int i = 0; i < nWeights; i++) {
			weights[i].position = (weights[i].position * (1 - strengthFrac)) + (((hammer[index][i] * (1 - indexFrac)) + (hammer[(index+1)%4][i] * indexFrac)) * strengthFrac);
			DEBUG("weight %f", weights[i].position);
		}
		
	}

	void inject() {}
};


struct FDM : Module {
	enum ParamIds {
		COARSE_PARAM,
		FINE_PARAM,
		RATE_PARAM,
		ATTEN_PARAM,
		SHAPE_PARAM,
		STRENGTH_PARAM,
		MASS_PARAM,
		SPRING_PARAM,
		CENTER_PARAM,
		DAMP_PARAM,
		GATE_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		PITCH_INPUT,
		RATE_INPUT,
		SHAPE_INPUT,
		STRENGTH_INPUT,
		MASS_INPUT,
		SPRING_INPUT,
		CENTER_INPUT,
		DAMP_INPUT,
		INJECT_INPUT,
		GATE_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		OSC_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		BLINK_LIGHT,
		NUM_LIGHTS
	};

	dsp::SchmittTrigger gateTrigger;
	dsp::SchmittTrigger buttonTrigger;

	MassSystem oscillator;

	float phase = 0.0f;
	float blinkPhase = 0.0f;
	float updatePhase = 0.0f;

	FDM() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		configParam(COARSE_PARAM, -3.0, 3.0, 0.0, "Coarse pitch control", " Volts");
		configParam(FINE_PARAM, -1.0, 1.0, 0.0, "Fine pitch control", " Volts");
		configParam(RATE_PARAM, 10.0, 1000.0, 0.0, "Simulation update rate", " Hz");
		configParam(ATTEN_PARAM, -1.0, 1.0, 0.0, "Attenuator", "x");
		configParam(SHAPE_PARAM, 0.0, 4.0, 0.0, "Excitation shape", "");
		configParam(STRENGTH_PARAM, 0.0, 1.0, 0.0, "Excitation strength", "");
		configParam(MASS_PARAM, 0.01, 5.0, 0.1, "Mass", " Kg");
		configParam(SPRING_PARAM, 0.0, 5.0, 0.0, "Inter-weight spring strength", "???");
		configParam(CENTER_PARAM, 0.0, 5.0, 0.0, "Centering spring strength", "???");
		configParam(DAMP_PARAM, 0.0, 1.0, 0.0, "Vertical damping force applied to weights", "???");

		configButton(GATE_PARAM, "Gate");

		configInput(PITCH_INPUT, "Pitch");
		configInput(RATE_INPUT, "Rate");
		configInput(SHAPE_INPUT, "Shape");
		configInput(STRENGTH_INPUT, "Strength");
		configInput(MASS_INPUT, "Mass");
		configInput(SPRING_INPUT, "Spring");
		configInput(CENTER_INPUT, "Center");
		configInput(DAMP_INPUT, "Damp");
		configInput(INJECT_INPUT, "Inject");
		configInput(GATE_INPUT, "Gate");
		
		configOutput(OSC_OUTPUT, "Oscillator");

		configLight(BLINK_LIGHT, "Excite");

		onReset();
		oscillator.initHammer();
	}
	//void step() override;



		//void FDM::step() {
	void process(const ProcessArgs& args) override {
		// Check current sample rate
		float deltaTime = args.sampleTime;

		// Compute frequency
		float pitch = params[COARSE_PARAM].getValue() + params[FINE_PARAM].getValue() + inputs[PITCH_INPUT].getVoltage();
		pitch = clamp(pitch, -4.0f, 4.0f);
		// The default pitch is C4
		float freq = 261.626f * powf(2.0f, pitch);

		// Accumulate the phase
		phase += freq * deltaTime;
		if (phase >= 1.0f)
			phase -= 1.0f;

		// Check trigger inputs
		if ((gateTrigger.process(inputs[GATE_INPUT].getVoltage())) | (buttonTrigger.process(params[GATE_PARAM].getValue()))) {
			oscillator.strike((params[STRENGTH_PARAM].getValue() + inputs[STRENGTH_INPUT].getVoltage()), (params[SHAPE_PARAM].getValue() + inputs[SHAPE_INPUT].getVoltage()));
		}

		// Coordinate state update
		updatePhase += deltaTime * params[RATE_PARAM].getValue();
		if (updatePhase >= 1.0) {
			oscillator.setMass(params[MASS_PARAM].getValue() + inputs[MASS_INPUT].getVoltage());
			oscillator.setDamp(params[DAMP_PARAM].getValue() + inputs[DAMP_INPUT].getVoltage());
			oscillator.setSpring(params[SPRING_PARAM].getValue() + inputs[SPRING_INPUT].getVoltage());
			oscillator.setCenter(params[CENTER_PARAM].getValue() + inputs[CENTER_INPUT].getVoltage());
			oscillator.update(1.0 / params[RATE_PARAM].getValue());
			updatePhase -= 1.0;
		}

		// Sample amplitude of mass-spring network
		//outputs[OSC_OUTPUT].setVoltage(clamp(5.0f * oscillator.sample(phase), -5.0, 5.0));
		float volts = clamp(5.0f * oscillator.sample(phase), -5.0, 5.0);
		outputs[OSC_OUTPUT].setVoltage(volts);
		//DEBUG("amp %f", volts);

		// Blink light
		lights[BLINK_LIGHT].setBrightness(((inputs[GATE_INPUT].getVoltage() == 1.0) || (params[GATE_PARAM].getValue() == 1.0)) ? 1.0 : 0.0);
	}

};

struct FDMWidget : ModuleWidget {
	FDMWidget(FDM* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/FDM.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<Rogan3PWhite>(Vec(45, 55), module, FDM::COARSE_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(45, 140), module, FDM::FINE_PARAM));
		
		addParam(createParamCentered<Rogan2PWhite>(Vec(110, 55), module, FDM::SHAPE_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(110, 140), module, FDM::STRENGTH_PARAM));
	
		addParam(createParamCentered<Rogan2PWhite>(Vec(170, 55), module, FDM::RATE_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(170, 140), module, FDM::ATTEN_PARAM));
		
		addParam(createParamCentered<Rogan2PWhite>(Vec(48, 210), module, FDM::MASS_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(89, 210), module, FDM::DAMP_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(130, 210), module, FDM::SPRING_PARAM));
		addParam(createParamCentered<Rogan2PWhite>(Vec(170, 210), module, FDM::CENTER_PARAM));
		addParam(createParamCentered<CKD6>(Vec(20, 325), module, FDM::GATE_PARAM));

		addInput(createInputCentered<PJ301MPort>(Vec(20, 289), module, FDM::GATE_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(55, 289), module, FDM::INJECT_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(55, 325), module, FDM::PITCH_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(90, 289), module, FDM::SHAPE_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(90, 325), module, FDM::STRENGTH_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(125, 289), module, FDM::MASS_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(125, 325), module, FDM::DAMP_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(160, 289), module, FDM::SPRING_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(160, 325), module, FDM::CENTER_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(195, 289), module, FDM::RATE_INPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(195, 325), module, FDM::OSC_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(Vec(41, 59), module, FDM::BLINK_LIGHT));
	}
};


// Specify the Module and ModuleWidget subclass, human-readable
// author name for categorization per plugin, module slug (should never
// change), human-readable module name, and any number of tags
// (found in `include/tags.hpp`) separated by commas.
//Model* modelFDM = Model::create<FDM, FDMWidget>("Metastable", "Metastable-FDM", "FDM", OSCILLATOR_TAG, PHYSICAL_MODELING_TAG);
Model* modelFDM = createModel<FDM, FDMWidget>("FDM");

