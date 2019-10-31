using UnityEngine;
using System.Collections;

namespace Pathfinding {
	
	[UniqueComponent(tag = "ai.destination")]
	public class Patrol : VersionedMonoBehaviour {
		public Transform[] targets;

		public float delay = 0;

		int index;

		IAstarAI agent;
		float switchTime = float.PositiveInfinity;

		protected override void Awake () {
			base.Awake();
			agent = GetComponent<IAstarAI>();
		}

		void Update () {
			if (targets.Length == 0) return;

			bool search = false;

			
			if (agent.reachedEndOfPath && !agent.pathPending && float.IsPositiveInfinity(switchTime)) {
				switchTime = Time.time + delay;
			}

			if (Time.time >= switchTime) {
				index = index + 1;
				search = true;
				switchTime = float.PositiveInfinity;
			}

			index = index % targets.Length;
			agent.destination = targets[index].position;

			if (search) agent.SearchPath();
		}
	}
}
