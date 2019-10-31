using UnityEngine;
using System.Collections.Generic;
using Pathfinding.Util;

namespace Pathfinding {
	[AddComponentMenu("Pathfinding/Modifiers/Funnel")]
	[System.Serializable]
	
	public class FunnelModifier : MonoModifier {
		
		public bool unwrap = true;

		
		public bool splitAtEveryPortal;

	#if UNITY_EDITOR
		[UnityEditor.MenuItem("CONTEXT/Seeker/Add Funnel Modifier")]
		public static void AddComp (UnityEditor.MenuCommand command) {
			(command.context as Component).gameObject.AddComponent(typeof(FunnelModifier));
		}
	#endif

		public override int Order { get { return 10; } }

		public override void Apply (Path p) {
			if (p.path == null || p.path.Count == 0 || p.vectorPath == null || p.vectorPath.Count == 0) {
				return;
			}

			List<Vector3> funnelPath = ListPool<Vector3>.Claim();

			// Split the path into different parts (separated by custom links)
			// and run the funnel algorithm on each of them in turn
			var parts = Funnel.SplitIntoParts(p);

			if (parts.Count == 0) {
				// As a really special case, it might happen that the path contained only a single node
				// and that node was part of a custom link (e.g added by the NodeLink2 component).
				// In that case the SplitIntoParts method will not know what to do with it because it is
				// neither a link (as only 1 of the 2 nodes of the link was part of the path) nor a normal
				// path part. So it will skip it. This will cause it to return an empty list.
				// In that case we want to simply keep the original path, which is just a single point.
				return;
			}

			for (int i = 0; i < parts.Count; i++) {
				var part = parts[i];
				if (!part.isLink) {
					var portals = Funnel.ConstructFunnelPortals(p.path, part);
					var result = Funnel.Calculate(portals, unwrap, splitAtEveryPortal);
					funnelPath.AddRange(result);
					ListPool<Vector3>.Release(ref portals.left);
					ListPool<Vector3>.Release(ref portals.right);
					ListPool<Vector3>.Release(ref result);
				} else {
					// non-link parts will add the start/end points for the adjacent parts.
					// So if there is no non-link part before this one, then we need to add the start point of the link
					// and if there is no non-link part after this one, then we need to add the end point.
					if (i == 0 || parts[i-1].isLink) {
						funnelPath.Add(part.startPoint);
					}
					if (i == parts.Count - 1 || parts[i+1].isLink) {
						funnelPath.Add(part.endPoint);
					}
				}
			}

			UnityEngine.Assertions.Assert.IsTrue(funnelPath.Count >= 1);
			ListPool<Funnel.PathPart>.Release(ref parts);
			// Pool the previous vectorPath
			ListPool<Vector3>.Release(ref p.vectorPath);
			p.vectorPath = funnelPath;
		}
	}
}
