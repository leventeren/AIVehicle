using UnityEngine;

public class AICarController : MonoBehaviour
{
    [Header("Araba Fizik Özellikleri")]
    [SerializeField] private Rigidbody carRigidbody;
    [SerializeField] private float motorTorque = 1500f;
    [SerializeField] private float brakeTorque = 3000f;
    [SerializeField] private float maxSpeed = 30f;
    
    [Header("Ackermann Direksiyon Ayarları")]
    [SerializeField] private Transform frontLeftWheel;
    [SerializeField] private Transform frontRightWheel;
    [SerializeField] private Transform rearLeftWheel;
    [SerializeField] private Transform rearRightWheel;
    [SerializeField] private float wheelBase = 2.5f;
    [SerializeField] private float trackWidth = 1.5f;
    [SerializeField] private float maxSteeringAngle = 30f;
    [SerializeField] private float steeringSpeed = 5f;
    
    [Header("AI Yol Takibi")]
    [SerializeField] private Transform[] waypointPath;
    [SerializeField] private float waypointReachDistance = 5f;
    [SerializeField] private float lookAheadDistance = 10f;
    [SerializeField] private int lookAheadWaypoints = 3;
    
    [Header("AI Davranış Parametreleri")]
    [SerializeField] private float steeringP = 2f;
    [SerializeField] private float steeringD = 1f;
    [SerializeField] private float speedAdjustmentFactor = 0.5f;
    [SerializeField] private bool useAckermann = true;
    
    [Header("Debug")]
    [SerializeField] private bool showDebugGizmos = true;
    [SerializeField] private float distanceToWaypoint;
    [SerializeField] private int currentWaypointIndex = 0;
    [SerializeField] private int lookaheadIndex;
    
    private float currentSteeringAngle = 0f;
    private float previousSteeringError = 0f;
    private Vector3 targetPosition;
    private WheelCollider[] wheelColliders = new WheelCollider[4];
    
    private void Start()
    {
        if (carRigidbody == null)
            carRigidbody = GetComponent<Rigidbody>();
        
        if (carRigidbody != null)
        {
            carRigidbody.mass = 1200f;
            carRigidbody.linearDamping = 0.05f;
            carRigidbody.angularDamping = 0.5f;
            carRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            carRigidbody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            carRigidbody.centerOfMass = new Vector3(0, -0.5f, 0);
        }
        
        SetupWheelColliders();
        
        if (waypointPath == null || waypointPath.Length == 0)
        {
            Debug.LogWarning("Waypoint yolu atanmamış!");
        }
    }
    
    private void SetupWheelColliders()
    {
        wheelColliders[0] = GetOrCreateWheelCollider(frontLeftWheel, "FL_WheelCollider");
        wheelColliders[1] = GetOrCreateWheelCollider(frontRightWheel, "FR_WheelCollider");
        wheelColliders[2] = GetOrCreateWheelCollider(rearLeftWheel, "RL_WheelCollider");
        wheelColliders[3] = GetOrCreateWheelCollider(rearRightWheel, "RR_WheelCollider");
    }
    
    private WheelCollider GetOrCreateWheelCollider(Transform wheelVisual, string colliderName)
    {
        if (wheelVisual == null) return null;
        
        var existing = wheelVisual.GetComponentInParent<WheelCollider>();
        if (existing == null)
            existing = wheelVisual.GetComponent<WheelCollider>();
        
        if (existing != null)
        {
            ConfigureWheelCollider(existing);
            return existing;
        }
        
        var colliderObj = new GameObject(colliderName);
        colliderObj.transform.SetParent(transform);
        colliderObj.transform.position = wheelVisual.position;
        colliderObj.transform.rotation = wheelVisual.rotation;
        
        var wc = colliderObj.AddComponent<WheelCollider>();
        ConfigureWheelCollider(wc);
        
        return wc;
    }
    
    private void ConfigureWheelCollider(WheelCollider wc)
    {
        wc.mass = 20f;
        wc.radius = 0.35f;
        wc.wheelDampingRate = 0.25f;
        wc.suspensionDistance = 0.2f;
        wc.forceAppPointDistance = 0f;
        
        var spring = wc.suspensionSpring;
        spring.spring = 20000f;
        spring.damper = 2000f;
        spring.targetPosition = 0.5f;
        wc.suspensionSpring = spring;
        
        var forwardFriction = wc.forwardFriction;
        forwardFriction.extremumSlip = 0.4f;
        forwardFriction.extremumValue = 1f;
        forwardFriction.asymptoteSlip = 0.8f;
        forwardFriction.asymptoteValue = 0.5f;
        forwardFriction.stiffness = 1f;
        wc.forwardFriction = forwardFriction;
        
        var sidewaysFriction = wc.sidewaysFriction;
        sidewaysFriction.extremumSlip = 0.2f;
        sidewaysFriction.extremumValue = 1f;
        sidewaysFriction.asymptoteSlip = 0.5f;
        sidewaysFriction.asymptoteValue = 0.75f;
        sidewaysFriction.stiffness = 1f;
        wc.sidewaysFriction = sidewaysFriction;
    }
    
    private void FixedUpdate()
    {
        if (waypointPath == null || waypointPath.Length == 0) return;
        
        UpdateTargetWaypoint();
        
        var steering = CalculateSteering();
        var throttle = CalculateThrottle();
        
        ApplyVehicleControl(steering, throttle);
    }
    
    private void UpdateTargetWaypoint()
    {
        distanceToWaypoint = Vector3.Distance(transform.position, waypointPath[currentWaypointIndex].position);
        
        if (distanceToWaypoint < waypointReachDistance)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % waypointPath.Length;
        }
        
        //targetPosition = waypointPath[currentWaypointIndex].position;
        
        //Revize
        lookaheadIndex = (currentWaypointIndex + lookAheadWaypoints) % waypointPath.Length;
        targetPosition = waypointPath[lookaheadIndex].position;
    }
    
    private float CalculateSteering()
    {
        var localTarget = transform.InverseTransformPoint(targetPosition);
        
        var targetAngle = Mathf.Atan2(localTarget.x, localTarget.z);
        
        var steeringError = targetAngle;
        var steeringDerivative = steeringError - previousSteeringError;
        previousSteeringError = steeringError;
        
        var desiredSteering = (steeringP * steeringError + steeringD * steeringDerivative) * Mathf.Rad2Deg;
        desiredSteering = Mathf.Clamp(desiredSteering, -maxSteeringAngle, maxSteeringAngle);
        
        currentSteeringAngle = Mathf.Lerp(currentSteeringAngle, desiredSteering, Time.fixedDeltaTime * steeringSpeed);
        
        return currentSteeringAngle;
    }
    
    private float CalculateThrottle()
    {
        var currentSpeed = carRigidbody.linearVelocity.magnitude;
        
        var steerFactor = 1f - (Mathf.Abs(currentSteeringAngle) / maxSteeringAngle) * speedAdjustmentFactor;
        var targetSpeed = maxSpeed * steerFactor;
        
        if (currentSpeed < targetSpeed)
        {
            return 1f;
        }
        else if (currentSpeed > targetSpeed + 2f)
        {
            return -1f;
        }
        
        return 0f;
    }
    
    private void ApplyVehicleControl(float steering, float throttle)
    {
        if (useAckermann)
        {
            ApplyAckermannSteering(steering);
        }
        else
        {
            if (wheelColliders[0] != null) wheelColliders[0].steerAngle = steering;
            if (wheelColliders[1] != null) wheelColliders[1].steerAngle = steering;
        }

        var motorForce = throttle * motorTorque;
        var brakeForce = throttle < 0 ? brakeTorque : 0f;

        for (var i = 0; i < 4; i++)
        {
            if (wheelColliders[i] != null)
            {
                wheelColliders[i].motorTorque = motorForce;
                wheelColliders[i].brakeTorque = brakeForce;
            }
        }

        UpdateWheelVisuals();
    }
    
    private void ApplyAckermannSteering(float steeringAngle)
    {
        if (Mathf.Approximately(steeringAngle, 0f))
        {
            if (wheelColliders[0] != null) wheelColliders[0].steerAngle = 0f;
            if (wheelColliders[1] != null) wheelColliders[1].steerAngle = 0f;
            return;
        }
        
        var steeringRad = steeringAngle * Mathf.Deg2Rad;
        
        var turnRadius = wheelBase / Mathf.Tan(Mathf.Abs(steeringRad));

        float leftAngle, rightAngle;
        
        if (steeringAngle > 0)
        {
            rightAngle = Mathf.Atan(wheelBase / (turnRadius - trackWidth / 2f)) * Mathf.Rad2Deg;
            leftAngle = Mathf.Atan(wheelBase / (turnRadius + trackWidth / 2f)) * Mathf.Rad2Deg;
        }
        else
        {
            leftAngle = -Mathf.Atan(wheelBase / (turnRadius - trackWidth / 2f)) * Mathf.Rad2Deg;
            rightAngle = -Mathf.Atan(wheelBase / (turnRadius + trackWidth / 2f)) * Mathf.Rad2Deg;
        }

        if (wheelColliders[0] != null) wheelColliders[0].steerAngle = leftAngle;
        if (wheelColliders[1] != null) wheelColliders[1].steerAngle = rightAngle;
    }
    
    private void UpdateWheelVisuals()
    {
        UpdateWheelVisual(wheelColliders[0], frontLeftWheel);
        UpdateWheelVisual(wheelColliders[1], frontRightWheel);
        UpdateWheelVisual(wheelColliders[2], rearLeftWheel);
        UpdateWheelVisual(wheelColliders[3], rearRightWheel);
    }
    
    private void UpdateWheelVisual(WheelCollider collider, Transform wheelTransform)
    {
        if (collider == null || wheelTransform == null) return;
        
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        
        wheelTransform.position = position;
        wheelTransform.rotation = rotation;
    }
    
    private void OnDrawGizmos()
    {
        if (!showDebugGizmos || waypointPath == null) return;
        
        Gizmos.color = Color.yellow;
        for (var i = 0; i < waypointPath.Length; i++)
        {
            if (waypointPath[i] == null) continue;
            
            var currentPos = waypointPath[i].position;
            var nextPos = waypointPath[(i + 1) % waypointPath.Length].position;
            
            Gizmos.DrawSphere(currentPos, 0.3f);
            Gizmos.DrawLine(currentPos, nextPos);
        }
        
        if (Application.isPlaying && waypointPath.Length > 0)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(waypointPath[currentWaypointIndex].position, 0.6f);
            Gizmos.DrawLine(transform.position, waypointPath[currentWaypointIndex].position);
        }
        
        if (Application.isPlaying && waypointPath.Length > 0)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(targetPosition, 0.5f);
            Gizmos.DrawLine(transform.position, targetPosition);
            
            Gizmos.color = Color.green;
            Gizmos.DrawLine(waypointPath[currentWaypointIndex].position, targetPosition);
        }
        
        if (Application.isPlaying && !Mathf.Approximately(currentSteeringAngle, 0f))
        {
            var steeringRad = currentSteeringAngle * Mathf.Deg2Rad;
            var turnRadius = wheelBase / Mathf.Tan(Mathf.Abs(steeringRad));
            
            var turnCenter = transform.position + transform.right * turnRadius * Mathf.Sign(currentSteeringAngle);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(turnCenter, turnRadius);
        }
    }
}