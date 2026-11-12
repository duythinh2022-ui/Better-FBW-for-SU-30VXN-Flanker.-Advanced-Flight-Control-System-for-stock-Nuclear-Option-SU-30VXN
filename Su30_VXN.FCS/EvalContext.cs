using UnityEngine;

namespace Su30_VXN.FCS
{
	internal struct EvalContext
	{
		public Vector3 cg;

		public Quaternion rootInv;

		public float rho;

		public Vector3 wind;

		// 2.0.8: the aircraft's air velocity (world), the reference flow for the stab downwash
		public Vector3 vAir;
	}
}
