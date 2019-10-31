using UnityEngine;

namespace Pathfinding.Examples {
	
	public class MineBotAI : AIPath {
		/// <summary>
		/// Animation component.
		/// Should hold animations "awake" and "forward"
		/// </summary>
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

#if UNITY_EDITOR
		protected override int OnUpgradeSerializedData (int version, bool unityThread) {
			if (unityThread) {
				var components = gameObject.GetComponents<Component>();
				int index = System.Array.IndexOf(components, this);
				foreach (System.Type newType in new [] { typeof(AIPath), typeof(MineBotAnimation) }) {
					var newComp = gameObject.AddComponent(newType);
					foreach (var field in newComp.GetType().GetFields(System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public)) {
						var oldField = this.GetType().GetField(field.Name);
						if (oldField != null) field.SetValue(newComp, oldField.GetValue(this));
					}
					for (int i = components.Length - 1; i > index; i--) UnityEditorInternal.ComponentUtility.MoveComponentUp(newComp);
				}
				GameObject.DestroyImmediate(this);
				return 0;
			} else {
				return base.OnUpgradeSerializedData(version, unityThread);
			}
		}
#endif
	}
}
