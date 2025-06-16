#pragma once


namespace Scene_constants {
	enum class translation {
		frame_static,
		translation_left,
		rotation_clockwise,
		trans_rAndRot_clock,
		max_elemtens_t,
	};

	enum class velocity {
		slow,
		medium,
		fast,
		max_elements_v,
	};

	enum class lightning {
		lambertian_basic,
		singel_light_source,
		max_elements_l,
	};

	enum class camera_para {
		store_events,
		store_frameAndEvents,
		max_element_c,
	};
}
