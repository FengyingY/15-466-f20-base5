#include "Mode.hpp"

#include "Scene.hpp"
#include "Sound.hpp"
#include "WalkMesh.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;

	void restart();
	//----- game state -----

	//input tracking:
	struct Button {
		uint8_t downs = 0;
		uint8_t pressed = 0;
	} left, right, down, up, dig;

	float wait = -1.f;
	float wait_limit = 2.f;

	//local copy of the game scene (so code can change it during gameplay):
	Scene scene;

	float detect_dist = 1.f;
	float sound_dist = 5.f;

	glm::vec3 initial_player_stand;

	Sound::PlayingSample *bomb_sound;

	//player info:
	struct Player {
		WalkPoint at;
		//transform is at player's feet and will be yawed by mouse left/right motion:
		Scene::Transform *transform = nullptr;
		//camera is at player's head and will be pitched by mouse up/down motion:
		Scene::Camera *camera = nullptr;
	} player;

	struct Target {
		Scene::Transform *transform = nullptr;
		bool found;
		std::string name;
	};

	std::vector<Target> target_vector;
	int remaining_target_count = 0;

	bool game_ends = false;

};
