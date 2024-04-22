(in-package #:org.shirakumo.fraf.trial)

(defstruct (animation-layer
            (:constructor %make-animation-layer (clip pose base)))
  (clip NIL :type clip)
  (pose NIL :type pose)
  (base NIL :type pose)
  (strength 0.0 :type single-float))

(defun make-animation-layer (clip skeleton &key (strength 0.0))
  (let ((layer (%make-animation-layer
                clip
                (rest-pose* skeleton)
                (instantiate-clip skeleton clip))))
    (setf (strength layer) strength)
    layer))

(defmethod strength ((layer animation-layer))
  (animation-layer-strength layer))

(defmethod (setf strength) (strength (layer animation-layer))
  (let ((clip (animation-layer-clip layer))
        (strength (clamp 0.0 (float strength 0f0) 1.0)))
    (sample (animation-layer-pose layer) clip (+ (start-time clip) (* strength (duration clip))))
    (setf (animation-layer-strength layer) strength)))

(defclass layer-controller ()
  ((animation-layers :initform (make-hash-table :test 'equalp) :accessor animation-layers)
   (pose :accessor pose)
   (skeleton :initform NIL :accessor skeleton)))

(defmethod shared-initialize :after ((controller layer-controller) slots &key animation-layers)
  (loop for layer in animation-layers
        for (clip . args) = (enlist layer)
        do (apply #'add-layer clip controller args)))

(defmethod describe-object :after ((controller layer-controller) stream)
  (terpri stream)
  (format stream "Layers:~%")
  (let ((layers (sort (alexandria:hash-table-keys (animation-layers controller)) #'string<)))
    (if layers
        (loop for name in layers
              for layer = (layer name controller)
              do (format stream "  ~3d% ~s~%" (round (* 100 (animation-layer-strength layer))) name))
        (format stream "  No layers.~%"))))

(defmethod update ((controller layer-controller) tt dt fc)
  (when (next-method-p) (call-next-method))
  (loop for layer being the hash-values of (animation-layers controller)
        do (layer-onto (pose controller) (pose controller) (animation-layer-pose layer) (animation-layer-base layer))))

(defmethod add-animation-layer ((layer animation-layer) (controller layer-controller) &key name)
  (setf (layer name controller) layer))

(defmethod add-animation-layer ((clip clip) (controller layer-controller) &key (strength 0.0) (name (name clip)))
  (setf (layer name controller) (make-animation-layer clip (skeleton controller) :strength strength)))

(defmethod remove-animation-layer (name (controller layer-controller))
  (setf (layer name controller) NIL))

(defmethod animation-layer (name (controller layer-controller))
  (gethash name (animation-layers controller)))

(defmethod (setf animation-layer) ((layer animation-layer) name (controller layer-controller))
  (setf (gethash name (animation-layers controller)) layer))

(defmethod (setf animation-layer) ((null null) name (controller layer-controller))
  (remhash name (animation-layers93 controller))
  null)

(defstruct (fade-target
            (:constructor make-fade-target (clip pose duration)))
  (pose NIL :type pose)
  (clip NIL :type clip)
  (clock 0.0 :type single-float)
  (duration 0.0 :type single-float)
  (elapsed 0.0 :type single-float))

(defmethod pose ((target fade-target)) (fade-target-pose target))
(defmethod clip ((target fade-target)) (fade-target-clip target))
(defmethod clock ((target fade-target)) (fade-target-clock target))
(defmethod duration ((target fade-target)) (fade-target-duration target))
(defmethod elapsed ((target fade-target)) (fade-target-elapsed target))

(defclass fade-controller ()
  ((fade-targets :initform (make-array 0 :adjustable T :fill-pointer T) :accessor fade-targets)
   (clip :initarg :clip :initform NIL :accessor clip)
   (clock :initform 0.0 :accessor clock)
   (playback-speed :initarg :playback-speed :initform 1.0 :accessor playback-speed)
   (pose :accessor pose)
   (skeleton :initform NIL :accessor skeleton)))

(defmethod shared-initialize :after ((controller fade-controller) slots &key skeleton)
  (when skeleton
    (setf (skeleton controller) skeleton)))

(defmethod (setf skeleton) :after ((skeleton skeleton) (controller fade-controller))
  (setf (pose controller) (rest-pose* skeleton)))

(defmethod describe-object :after ((controller fade-controller) stream)
  (terpri stream)
  (format stream "Current Clip:~%")
  (if (clip controller)
      (format stream "  ~4f / ~4f ~s~%"
              (clock controller) (duration (clip controller)) (name (clip controller)))
      (format stream "  No current clip.~%"))
  (terpri stream)
  (format stream "Fade Targets:~%")
  (if (< 0 (length (fade-targets controller)))
      (loop for target across (fade-targets controller)
            do (format stream "  ~4f / ~4f ~s~%"
                       (fade-target-clock target) (fade-target-duration target)
                       (name (fade-target-clip target))))
      (format stream "  No current fade targets.~%")))

(defmethod play ((target clip) (controller fade-controller))
  (unless (eq target (clip controller))
    (setf (playback-speed controller) 1.0)
    (setf (fill-pointer (fade-targets controller)) 0)
    (setf (clip controller) target)
    (pose<- (pose controller) (rest-pose (skeleton controller)))
    (setf (clock controller) (start-time target))
    (sample (pose controller) (clip controller) (clock controller))))

(defmethod fade-to ((target clip) (controller fade-controller) &key (duration (blend-duration target)))
  (let ((targets (fade-targets controller)))
    (cond ((or (null (clip controller)) (<= duration 0))
           (play target controller))
          ((and (or (= 0 (length targets))
                    (not (eq target (fade-target-clip (aref targets (1- (length targets)))))))
                (not (eq target (clip controller))))
           (setf (playback-speed controller) 1.0)
           (vector-push-extend (make-fade-target target (rest-pose (skeleton controller)) (float duration 0f0)) targets)))))

(defmethod update ((controller fade-controller) tt dt fc)
  (when (next-method-p) (call-next-method))
  (let ((clip (clip controller)))
    (when (and clip (skeleton controller))
      (let ((targets (fade-targets controller)))
        (loop for target across targets
              for i from 0
              do (when (<= (fade-target-duration target) (fade-target-elapsed target))
                   (setf clip (setf (clip controller) (fade-target-clip target)))
                   (setf (clock controller) (fade-target-clock target))
                   (pose<- (pose controller) (fade-target-pose target))
                   (array-utils:vector-pop-position targets i)
                   (return)))
        (let ((time (sample (pose controller) clip (+ (clock controller) (* (playback-speed controller) dt)))))
          (setf (clock controller) time)
          (when (and (not (loop-p clip))
                     (<= (end-time clip) time)
                     (next-clip clip))
            (fade-to (next-clip clip) controller))
          (loop for target across targets
                do (setf (fade-target-clock target) (sample (fade-target-pose target) (fade-target-clip target) (+ (fade-target-clock target) dt)))
                   (incf (fade-target-elapsed target) dt)
                   (let ((time (min 1.0 (/ (fade-target-elapsed target) (fade-target-duration target)))))
                     (blend-into (pose controller) (pose controller) (fade-target-pose target) time))))))))

(defclass animation-controller (ik-controller layer-controller fade-controller listener)
  ((model :initform NIL :accessor model)
   (updated-on :initform -1 :accessor updated-on)
   (palette :initform #() :accessor palette)
   (palette-texture :initform (make-instance 'texture :target :texture-1d-array :width 3 :height 1 :internal-format :rgba32f :min-filter :nearest :mag-filter :nearest) :accessor palette-texture)
   (palette-data :initform (make-array 0 :element-type 'single-float) :accessor palette-data)))

(defmethod describe-object :after ((entity animation-controller) stream)
  (terpri stream)
  (format stream "Available Clips:~%")
  (if (list-clips entity)
      (loop for clip in (list-clips entity)
            do (format stream "  ~s~%" clip))
      (format stream "  No currently available clips.~%"))
  (terpri stream)
  (format stream "Skeleton:~%")
  (describe-skeleton (skeleton entity) stream))

(defmethod observe-load-state ((entity animation-controller) (asset model) (state (eql :loaded)) (area staging-area))
  (setf (model entity) asset))

(defmethod (setf model) :after ((asset model-file) (entity animation-controller))
  (when (loaded-p asset)
    (setf (skeleton entity) (skeleton asset)))
  (play (or (clip entity) T) entity))

(defmethod find-clip (name (entity animation-controller) &optional (errorp T))
  (if (null (model entity))
      (when errorp (error "No such clip ~s found on ~a" name entity))
      (find-clip name (model entity) errorp)))

(defmethod list-clips ((entity animation-controller))
  (when (model entity)
    (list-clips (model entity))))

(defmethod add-layer (clip-name (entity animation-controller) &key (name NIL name-p))
  (let ((clip (find-clip clip-name entity)))
    (add-layer clip entity :name (if name-p name (name clip)))))

(defmethod fade-to ((name string) (entity animation-controller) &rest args &key &allow-other-keys)
  (apply #'fade-to (find-clip name entity) entity args))

(defmethod fade-to ((name symbol) (entity animation-controller) &rest args &key &allow-other-keys)
  (apply #'fade-to (find-clip name entity) entity args))

(defmethod play ((name string) (entity animation-controller))
  (play (find-clip name entity) entity))

(defmethod play ((name symbol) (entity animation-controller))
  (play (find-clip name entity) entity))

(defmethod play ((anything (eql T)) (entity animation-controller))
  (loop for clip being the hash-values of (clips (model entity))
        do (return (play clip entity))))

(defmethod update ((entity animation-controller) tt dt fc)
  (when (/= (updated-on entity) fc)
    (call-next-method)
    (update-palette entity)
    (setf (updated-on entity) fc)))

(defmethod stage :after ((entity animation-controller) (area staging-area))
  (stage (palette-texture entity) area))

(defmethod (setf pose) :after ((pose pose) (entity animation-controller))
  (update-palette entity))

(defmethod (setf ik-system) :after ((system ik-system) name (entity animation-controller))
  ;; Hook up our local transform to the IK system's. Since the identity never changes
  ;; the properties "transfer".
  (setf (slot-value system 'transform) (tf entity)))

(define-handler ((entity animation-controller) (ev tick)) (tt dt fc)
  (update entity tt dt fc))

(defmethod update-palette ((entity animation-controller))
  (let* ((palette (matrix-palette (pose entity) (palette entity)))
         (texinput (%adjust-array (palette-data entity) (* 12 (length (pose entity))) (constantly 0f0)))
         (texture (palette-texture entity))
         (inv (mat-inv-bind-pose (skeleton (model entity)))))
    (mem:with-memory-region (texptr texinput)
      (dotimes (i (length palette) (setf (palette entity) palette))
        (let ((mat (nm* (svref palette i) (svref inv i)))
              (texptr (mem:memory-region-pointer texptr)))
          (cffi:with-pointer-to-vector-data (arrptr (marr4 mat))
            (static-vectors:replace-foreign-memory
             (cffi:inc-pointer texptr (* i 12 4)) arrptr (* 12 4))))))
    (setf (palette-data entity) texinput)
    (setf (height texture) (length palette))
    (when (gl-name texture)
      (resize-buffer-data texture texinput :pixel-type :float :pixel-format :rgba))))

(defmethod instantiate-prefab :before ((instance animation-controller) (asset model))
  (setf (model instance) asset))

(defmethod instantiate-prefab :after ((instance animation-controller) asset)
  (do-scene-graph (child instance)
    (when (typep child 'base-animated-entity)
      (setf (animation-controller child) instance))))

(define-shader-entity base-animated-entity (mesh-entity)
  ((animation-controller :initform (make-instance 'animation-controller) :accessor animation-controller)))

(define-transfer base-animated-entity animation-controller)

(define-accessor-wrapper-methods clip (base-animated-entity (animation-controller base-animated-entity)))
(define-accessor-wrapper-methods skeleton (base-animated-entity (animation-controller base-animated-entity)))
(define-accessor-wrapper-methods pose (base-animated-entity (animation-controller base-animated-entity)))
(define-accessor-wrapper-methods palette (base-animated-entity (animation-controller base-animated-entity)))
(define-accessor-wrapper-methods palette-texture (base-animated-entity (animation-controller base-animated-entity)))

(defmethod (setf mesh-asset) :after ((asset model-file) (entity base-animated-entity))
  (setf (model (animation-controller entity)) asset))

(defmethod stage :after ((entity base-animated-entity) (area staging-area))
  (register-load-observer area (animation-controller entity) (mesh-asset entity))
  (stage (animation-controller entity) area))

(defmethod play (thing (entity base-animated-entity))
  (play thing (animation-controller entity)))

(defmethod fade-to (thing (entity base-animated-entity) &rest args &key &allow-other-keys)
  (apply #'fade-to thing (animation-controller entity) args))

(defmethod add-layer (thing (entity base-animated-entity) &rest args)
  (apply #'add-layer thing (animation-controller entity) args))

(defmethod find-clip (thing (entity base-animated-entity) &optional (errorp T))
  (find-clip thing (animation-controller entity) errorp))

(defmethod list-clips ((entity base-animated-entity))
  (list-clips entity))

(define-shader-entity armature (base-animated-entity lines)
  ((color :initarg :color :initform (vec 0 0 0 1) :accessor color)))

(define-handler ((entity armature) (ev tick) :after) ()
  (replace-vertex-data entity (pose entity) :default-color (color entity)))

(define-shader-entity animated-entity (base-animated-entity transformed-entity)
  ((mesh :initarg :mesh :initform NIL :accessor mesh)))

(defmethod (setf mesh-asset) :after ((asset model-file) (entity animated-entity))
  (unless (loaded-p asset)
    (setf (palette entity) #(#.(meye 4)))))

(defmethod render :before ((entity animated-entity) (program shader-program))
  (declare (optimize speed))
  ;; KLUDGE: This is Bad
  (%gl:active-texture :texture5)
  (gl:bind-texture :texture-1d-array (gl-name (palette-texture entity)))
  (setf (uniform program "pose") 5))

(define-class-shader (animated-entity :vertex-shader)
  "
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 in_normal;
layout (location = 2) in vec2 in_uv;
layout (location = 3) in vec4 joints;
layout (location = 4) in vec4 weights;

uniform sampler1DArray pose;

out vec3 normal;
out vec4 world_pos;
out vec2 uv;

mat4 pose_matrix(in int i){
  return transpose(mat4(
    texelFetch(pose, ivec2(0, i), 0),
    texelFetch(pose, ivec2(1, i), 0),
    texelFetch(pose, ivec2(2, i), 0),
    vec4(0,0,0,1)));
}

void main(){
  ivec4 j = ivec4(joints);
  mat4 skin_matrix = (pose_matrix(j.x) * weights.x)
                   + (pose_matrix(j.y) * weights.y)
                   + (pose_matrix(j.z) * weights.z)
                   + (pose_matrix(j.w) * weights.w);
  world_pos = model_matrix * skin_matrix * vec4(position, 1.0f);
  normal = vec3(model_matrix * skin_matrix * vec4(in_normal, 0.0f));
  uv = in_uv;
  gl_Position = projection_matrix * view_matrix * world_pos;
}")

(defclass quat2-animation-controller (animation-controller)
  ())

(defmethod update-palette ((entity quat2-animation-controller))
  ;; FIXME: Update for texture data
  (let ((palette (quat2-palette (pose entity) (palette entity)))
        (inv (quat-inv-bind-pose (skeleton (mesh-asset entity)))))
    (dotimes (i (length palette) (setf (palette entity) palette))
      (nq* (svref palette i) (svref inv i)))))

(define-shader-entity quat2-animated-entity (base-animated-entity)
  ())

(defmethod render :before ((entity quat2-animated-entity) (program shader-program))
  (declare (optimize speed))
  ;; KLUDGE: This is Bad
  (%gl:active-texture :texture5)
  (gl:bind-texture :texture-1d-array (gl-name (palette-texture entity)))
  (setf (uniform program "pose") 5))

(define-class-shader (quat2-animated-entity :vertex-shader)
  "
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 in_normal;
layout (location = 2) in vec2 in_uv;
layout (location = 3) in vec4 joints;
layout (location = 4) in vec4 weights;

uniform mat2x4 pose[120];

out vec3 normal;
out vec4 world_pos;
out vec2 uv;

vec4 quat_mul(vec4 q1, vec4 q2){
  return vec4(q2.x*q1.w + q2.y*q1.z - q2.z*q1.y + q2.w*q1.x,
             -q2.x*q1.z + q2.y*q1.w + q2.z*q1.x + q2.w*q1.y,
              q2.x*q1.y - q2.y*q1.x + q2.z*q1.w + q2.w*q1.z,
             -q2.x*q1.x - q2.y*q1.y - q2.z*q1.z + q2.w*q1.w);
}

vec4 dquat_vector(mat2x4 dq, vec3 v){
  vec4 real = dq[0];
  vec3 r_vector = real.xyz;
  float r_scalar = real.w;
  vec3 rotated = r_vector*2*dot(r_vector, v)
    + v*(r_scalar*r_scalar - dot(r_vector, r_vector))
    + cross(r_vector, v)*2*r_scalar;
  return vec4(rotated, 0);
}

vec4 dquat_point(mat2x4 dq, vec3 v){
  vec4 real = dq[0];
  vec4 dual = dq[1];
  vec3 rotated = dquat_vector(dq, v).xyz;
  vec4 conjugate = vec4(-real.xyz, real.w);
  vec3 t = quat_mul(conjugate, dual*2).xyz;
  return vec4(rotated+t, 1);
}

mat2x4 dquat_normalized(mat2x4 dq){
  float inv_mag = 1.0 / length(dq[0]);
  dq[0] *= inv_mag;
  dq[1] *= inv_mag;
  return dq;
}

void main(){
  ivec4 j = ivec4(joints);

  mat2x4 skin_dq = dquat_normalized(
    + (pose[j.x] * weights.x)
    + (pose[j.y] * weights.y)
    + (pose[j.z] * weights.z)
    + (pose[j.w] * weights.w));
  world_pos = model_matrix * dquat_point(skin_dq, position);
  normal = vec3(model_matrix * dquat_vector(skin_dq, in_normal));
  uv = in_uv;
  gl_Position = projection_matrix * view_matrix * world_pos;
}")
