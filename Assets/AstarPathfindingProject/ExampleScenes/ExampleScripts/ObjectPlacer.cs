using UnityEngine;
using System.Collections;

namespace Pathfinding.Examples {
	
	public class ObjectPlacer : MonoBehaviour {
		/// <summary>
		/// GameObject to place.
		/// When using a Grid Graph you need to make sure the object's layer is included in the collision mask in the GridGraph settings.
		/// </summary>
		public GameObject go;

		/// <summary>Flush Graph Updates directly after placing. Slower, but updates are applied immidiately</summary>
		public bool direct = false;

		/// <summary>Issue a graph update object after placement</summary>
		public bool issueGUOs = true;

		/// <summary>Update is called once per frame</summary>
		void Update () {
			if (Input.GetKeyDown("p")) {
				PlaceObject();
			}

			if (Input.GetKeyDown("r")) {
				StartCoroutine(RemoveObject());
			}
		}

		public void PlaceObject () {
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			RaycastHit hit;

			// Figure out where the ground is
			if (Physics.Raycast(ray, out hit, Mathf.Infinity)) {
				Vector3 p = hit.point;
				GameObject obj = GameObject.Instantiate(go, p, Quaternion.identity) as GameObject;

				if (issueGUOs) {
					Bounds b = obj.GetComponent<Collider>().bounds;
					GraphUpdateObject guo = new GraphUpdateObject(b);
					AstarPath.active.UpdateGraphs(guo);
					if (direct) {
						AstarPath.active.FlushGraphUpdates();
					}
				}
			}
		}

		public IEnumerator RemoveObject () {
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			RaycastHit hit;

			// Check what object is under the mouse cursor
			if (Physics.Raycast(ray, out hit, Mathf.Infinity)) {
				// Ignore ground and triggers
				if (hit.collider.isTrigger || hit.transform.gameObject.name == "Ground") yield break;

				Bounds b = hit.collider.bounds;
				Destroy(hit.collider);
				Destroy(hit.collider.gameObject);

				if (issueGUOs) {
					
					yield return new WaitForEndOfFrame();
					GraphUpdateObject guo = new GraphUpdateObject(b);
					AstarPath.active.UpdateGraphs(guo);
					if (direct) {
						AstarPath.active.FlushGraphUpdates();
					}
				}
			}
		}
	}
}
