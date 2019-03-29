#include "Metastable.hpp"
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
		float indexInt;
		float indexFrac = modf(indexMix, &indexInt);

		int32_t index = static_cast<int32_t>(indexInt);
		float amplitude = (weights[index].position * (1 - indexFrac)) + (weights[(index + 1) % nWeights].position * indexFrac);
		return amplitude;
	}
	
	void strike(float strength, float shape) {
		for (int i = 0; i < nWeights; i++) {
			weights[i].position = sinf(static_cast<float>(i) / static_cast<float>(nWeights) * 2 * M_PI);
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

	SchmittTrigger gateTrigger;
	SchmittTrigger buttonTrigger;

	float phase = 0.0;
	float blinkPhase = 0.0;
	float updatePhase = 0.0;

	FDM() : Module(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS) {}
	void step() override;

	MassSystem oscillator;

	// For more advanced Module features, read Rack's engine.hpp header file
	// - toJson, fromJson: serialization of internal data
	// - onSampleRateChange: event triggered by a change of sample rate
	// - onReset, onRandomize, onCreate, onDelete: implements special behavior when user clicks these from the context menu
};


void FDM::step() {
	// Check current sample rate
	float deltaTime = engineGetSampleTime();

	// Compute frequency
	float pitch = params[COARSE_PARAM].value + params[FINE_PARAM].value + inputs[PITCH_INPUT].value;
	pitch = clamp(pitch, -4.0f, 4.0f);
	// The default pitch is C4
	float freq = 261.626f * powf(2.0f, pitch);

	// Accumulate the phase
	phase += freq * deltaTime;
	if (phase >= 1.0f)
		phase -= 1.0f;

	// Check trigger inputs
	if ((gateTrigger.process(inputs[GATE_INPUT].value)) || (buttonTrigger.process(params[GATE_PARAM].value))) {
		oscillator.strike(1.0, 1.0);
	}

	// Coordinate state update
	updatePhase += deltaTime * params[RATE_PARAM].value;
	if (updatePhase >= 1.0) {
		oscillator.setMass(params[MASS_PARAM].value + inputs[MASS_INPUT].value);
		oscillator.setDamp(params[DAMP_PARAM].value + inputs[DAMP_INPUT].value);
		oscillator.setSpring(params[SPRING_PARAM].value + inputs[SPRING_INPUT].value);
		oscillator.setCenter(params[CENTER_PARAM].value + inputs[CENTER_INPUT].value);
		oscillator.update(1.0 / (params[RATE_PARAM].value * 10));
		updatePhase -= 1.0;
	}

	// Sample amplitude of mass-spring network
	outputs[OSC_OUTPUT].value = clamp(5.0f * oscillator.sample(phase), -5.0, 5.0);

	// Blink light
	lights[BLINK_LIGHT].value = ((inputs[GATE_INPUT].value == 1.0) || (params[GATE_PARAM].value == 1.0)) ? 1.0 : 0.0;
}


struct FDMWidget : ModuleWidget {
	FDMWidget(FDM *module) : ModuleWidget(module) {
		setPanel(SVG::load(assetPlugin(plugin, "res/FDM.svg")));

		addChild(Widget::create<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(Widget::create<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(Widget::create<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(Widget::create<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(ParamWidget::create<Rogan3PWhite>(Vec(45, 55), module, FDM::COARSE_PARAM, -3.0, 3.0, 0.0));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(45, 140), module, FDM::FINE_PARAM, -1.0, 1.0, 0));
		
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(110, 55), module, FDM::SHAPE_PARAM, 0.0, 4.0, 0.0));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(110, 140), module, FDM::STRENGTH_PARAM, 0.0, 1.0, 0.0));
	
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(170, 55), module, FDM::RATE_PARAM, 10.0, 10000.0, 0.0));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(170, 140), module, FDM::ATTEN_PARAM, -1.0, 1.0, 0));
		
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(48, 210), module, FDM::MASS_PARAM, 0.01, 5.0, 0.1));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(89, 210), module, FDM::DAMP_PARAM, 0.0, 1.0, 0));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(130, 210), module, FDM::SPRING_PARAM, 0.0, 5.0, 0));
		addParam(ParamWidget::create<Rogan2PWhite>(Vec(170, 210), module, FDM::CENTER_PARAM, 0.0, 5.0, 0));

		//addChild(ModuleLightWidget::create<MediumLight<RedLight>>(Vec(41, 59), module, FDM::BLINK_LIGHT));
	
		addInput(Port::create<PJ301MPort>(Vec(20, 289), Port::INPUT, module, FDM::GATE_INPUT));
		addParam(ParamWidget::create<CKD6>(Vec(20, 325), module, FDM::GATE_PARAM, 0.0, 1.0, 0.0));
		addInput(Port::create<PJ301MPort>(Vec(55, 289), Port::INPUT, module, FDM::INJECT_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(55, 325), Port::INPUT, module, FDM::PITCH_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(90, 289), Port::INPUT, module, FDM::SHAPE_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(90, 325), Port::INPUT, module, FDM::STRENGTH_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(125, 289), Port::INPUT, module, FDM::MASS_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(125, 325), Port::INPUT, module, FDM::DAMP_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(160, 289), Port::INPUT, module, FDM::SPRING_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(160, 325), Port::INPUT, module, FDM::CENTER_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(195, 289), Port::INPUT, module, FDM::RATE_INPUT));
		addOutput(Port::create<PJ301MPort>(Vec(195, 325), Port::OUTPUT, module, FDM::OSC_OUTPUT));
	}
};


// Specify the Module and ModuleWidget subclass, human-readable
// author name for categorization per plugin, module slug (should never
// change), human-readable module name, and any number of tags
// (found in `include/tags.hpp`) separated by commas.
Model *modelFDM = Model::create<FDM, FDMWidget>("Metastable", "Metastable-FDM", "FDM", OSCILLATOR_TAG, PHYSICAL_MODELING_TAG);

