using UnityEngine;

namespace Pathfinding.Examples {

	public class MineBotAnimation : VersionedMonoBehaviour {
		
		public Animation anim;

		/// <summary>Minimum velocity for moving</summary>
		public float sleepVelocity = 0.4F;

		/// <summary>Speed relative to velocity with which to play animations</summary>
		public float animationSpeed = 0.2F;

		/// <summary>
		/// Effect which will be instantiated when end of path is reached.
		/// See: OnTargetReached
		/// </summary>
		public GameObject endOfPathEffect;

		bool isAtDestination;

		IAstarAI ai;
		Transform tr;

		protected override void Awake () {
			base.Awake();
			ai = GetComponent<IAstarAI>();
			tr = GetComponent<Transform>();
		}

		void Start () {
			// Prioritize the walking animation
			anim["forward"].layer = 10;

			// Play all animations
			anim.Play("awake");
			anim.Play("forward");

			// Setup awake animations properties
			anim["awake"].wrapMode = WrapMode.Clamp;
			anim["awake"].speed = 0;
			anim["awake"].normalizedTime = 1F;
		}

		/// <summary>Point for the last spawn of <see cref="endOfPathEffect"/></summary>
		protected Vector3 lastTarget;

		/// <summary>
		/// Called when the end of path has been reached.
		/// An effect (<see cref="endOfPathEffect)"/> is spawned when this function is called
		/// However, since paths are recalculated quite often, we only spawn the effect
		/// when the current position is some distance away from the previous spawn-point
		/// </summary>
		void OnTargetReached () {
			if (endOfPathEffect != null && Vector3.Distance(tr.position, lastTarget) > 1) {
				GameObject.Instantiate(endOfPathEffect, tr.position, tr.rotation);
				lastTarget = tr.position;
			}
		}

		protected void Update () {
			if (ai.reachedEndOfPath) {
				if (!isAtDestination) OnTargetReached();
				isAtDestination = true;
			} else isAtDestination = false;

			// Calculate the velocity relative to this transform's orientation
			Vector3 relVelocity = tr.InverseTransformDirection(ai.velocity);
			relVelocity.y = 0;

			if (relVelocity.sqrMagnitude <= sleepVelocity*sleepVelocity) {
				// Fade out walking animation
				anim.Blend("forward", 0, 0.2F);
			} else {
				// Fade in walking animation
				anim.Blend("forward", 1, 0.2F);

				// Modify animation speed to match velocity
				AnimationState state = anim["forward"];

				float speed = relVelocity.z;
				state.speed = speed*animationSpeed;
			}
		}
	}
}
