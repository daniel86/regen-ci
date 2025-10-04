#ifndef REGEN_BLACKBOARD_H_
#define REGEN_BLACKBOARD_H_
#include "regen/utility/time.h"
#include "world/character-trait.h"
#include "world/world-model.h"

namespace regen {
	/**
	 * @brief A blackboard encapsulates NPC state, in a way what the NPC "knows" about the world.
	 * This can be used to implement more complex behaviors, e.g. using a behavior tree
	 * or a planning system.
	 */
	class Blackboard {
	public:
		Blackboard() {
			for (int i = 0; i < static_cast<int>(Trait::LAST_TRAIT); i++) {
				traits_[i] = 0.5f;
			}
		}

		void setWorldTime(const WorldTime *worldTime) { worldTime_ = worldTime; }

		const WorldTime *worldTime() const { return worldTime_; }

		void advanceTime(float dt_s) {
			lingerTime_ -= dt_s;
			activityTime_ -= dt_s;
		}

		void setPatient(const ref_ptr<WorldObject> &patientObj,
					const ref_ptr<Affordance> &aff,
					int slotIdx) {
			patient_.object = patientObj;
			patient_.affordance = aff;
			patient_.affordanceSlot = slotIdx;
		}

		void unsetPatient() {
			if (patient_.affordance.get()) {
				patient_.affordance->releaseSlot(patient_.affordanceSlot);
				patient_.affordance = {};
				patient_.affordanceSlot = -1;
			}
			patient_.object = {};
		}

		const Patient& patient() const { return patient_; }

		bool hasPatient() const { return patient_.object.get() != nullptr; }

		/**
		 * Add a place of interest for the NPC.
		 * @param place the place.
		 */
		void addPlaceOfInterest(const ref_ptr<Place> &place)  {
			switch (place->placeType()) {
				case PLACE_HOME:
					homePlaces_.push_back(place);
					break;
				case PLACE_SPIRITUAL:
					spiritualPlaces_.push_back(place);
					break;
				case PLACE_GATHERING:
					gatheringPlaces_.push_back(place);
					break;
				case PLACE_PATROL_POINT:
					patrolPoints_.push_back(place);
					break;
				default:
					REGEN_WARN("Unhandled place type " << place->placeType() << ".");
					break;
			}
		}

		const std::vector<ref_ptr<Place>> &homePlaces() const { return homePlaces_; }

		const std::vector<ref_ptr<Place>> &spiritualPlaces() const { return spiritualPlaces_; }

		const std::vector<ref_ptr<Place>> &gatheringPlaces() const { return gatheringPlaces_; }

		const ref_ptr<Place> &homePlace() const { return homePlace_; }

		void setHomePlace(const ref_ptr<Place> &place) { homePlace_ = place; }

		const ref_ptr<Place> &currentPlace() const { return currentPlace_; }

		void setCurrentPlace(const ref_ptr<Place> &place) { currentPlace_ = place; }

		void unsetCurrentPlace() { currentPlace_ = {}; }

		void setTargetPlace(const ref_ptr<Place> &place) { targetPlace_ = place; }

		const ref_ptr<Place> &targetPlace() const { return targetPlace_; }

		void setTraitStrength(Trait trait, float value) {
			traits_[static_cast<int>(trait)] = value;
		}

		float traitStrength(Trait trait) const {
			return traits_[static_cast<int>(trait)];
		}

		float laziness() const { return traits_[static_cast<int>(Trait::LAZINESS)]; }

		float spirituality() const { return traits_[static_cast<int>(Trait::SPIRITUALITY)]; }

		float alertness() const { return traits_[static_cast<int>(Trait::ALERTNESS)]; }

		float bravery() const { return traits_[static_cast<int>(Trait::BRAVERY)]; }

		float sociability() const { return traits_[static_cast<int>(Trait::SOCIALABILITY)]; }

		void setLingerTime(double time) { lingerTime_ = time; }

		float lingerTime() const { return lingerTime_; }

		void setActivityTime(double time) { activityTime_ = time; }

		float activityTime() const { return activityTime_; }

		/**
		 * Set the base time (in seconds) the NPC will spend on an activity.
		 * This is then modulated by personality traits and the type of activity.
		 * @param time the base time in seconds.
		 */
		void setBaseTimeActivity(float time) { baseTimeActivity_ = time; }

		float baseTimeActivity() const { return baseTimeActivity_; }

		/**
		 * Set the base time (in seconds) the NPC will spend at a place.
		 * This is then modulated by personality traits and the type of place.
		 * @param time the base time in seconds.
		 */
		void setBaseTimePlace(float time) { baseTimePlace_ = time; }

		float baseTimePlace() const { return baseTimePlace_; }

	protected:
		const WorldTime *worldTime_ = nullptr;

		// Some high level character traits.
		// These are used for weighting options.
		std::array<float, static_cast<int>(Trait::LAST_TRAIT)> traits_;

		// Places of interest.
		std::vector<ref_ptr<Place>> homePlaces_;
		std::vector<ref_ptr<Place>> spiritualPlaces_;
		std::vector<ref_ptr<Place>> gatheringPlaces_;
		std::vector<ref_ptr<Place>> patrolPoints_;

		ref_ptr<Place> homePlace_;
		ref_ptr<Place> targetPlace_;
		ref_ptr<Place> currentPlace_;

		// Base times for activities and places.
		// The actual time spent is modulated by personality traits and the type of activity/place
		float baseTimeActivity_ = 20.0f; // in seconds
		float baseTimePlace_ = 240.0f; // in seconds

		Patient patient_;
		float lingerTime_ = 0.0f;
		float activityTime_ = 0.0f;
	};
} // namespace

#endif /* REGEN_BLACKBOARD_H_ */
