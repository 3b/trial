(in-package #:org.shirakumo.fraf.trial)

(define-shader-entity debug-draw-text (standalone-shader-entity)
  ((text-vao :accessor text-vao)
   (text :accessor text)))

(defmethod initialize-instance :after ((draw debug-draw-text) &key)
  (setf (text draw) (make-array 256 :fill-pointer 0 :adjustable T :element-type 'single-float))
  (let ((vbo (make-instance 'vertex-buffer :buffer-data (text draw))))
    (setf (text-vao draw) (make-instance 'vertex-array :vertex-form :triangles :bindings
                                         `((,vbo :size 3 :offset 0 :stride 20 :index 0)
                                           (,vbo :size 2 :offset 12 :stride 20 :index 2))))))

(defmethod stage :after ((draw debug-draw-text) (area staging-area))
  (stage (text-vao draw) area)
  (stage (// 'trial 'ascii) area))

(defmethod render ((draw debug-draw-text) (program shader-program))
  (setf (uniform program "view_matrix") (view-matrix))
  (setf (uniform program "projection_matrix") (projection-matrix))
  (setf (uniform program "texture_image") 0)
  (gl:active-texture :texture0)
  (gl:bind-texture :texture-2d (gl-name (// 'trial 'ascii)))
  (render-array (text-vao draw) :vertex-count (truncate (length (text draw)) 5)))

(define-class-shader (debug-draw-text :vertex-shader)
  "layout (location = 0) in vec3 position;
layout (location = 2) in vec2 i_uv;
out vec2 v_uv;

uniform sampler2D texture_image;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main(){
  mat4 modelView = view_matrix;
  modelView[0][0] = 1.0;
  modelView[0][1] = 0.0;
  modelView[0][2] = 0.0;
  modelView[1][0] = 0.0;
  modelView[1][1] = 1.0;
  modelView[1][2] = 0.0;
  modelView[2][0] = 0.0;
  modelView[2][1] = 0.0;
  modelView[2][2] = 1.0;

  gl_Position = projection_matrix * modelView * vec4(position, 1.0f);
  gl_Position.z = -1.0;
  v_uv = i_uv / textureSize(texture_image, 0).rg;
}")

(define-class-shader (debug-draw-text :fragment-shader)
  "in vec2 v_uv;
out vec4 color;
uniform sampler2D texture_image;

void main(){
  float fg_bg = texture(texture_image, v_uv, 0).r;
  color = mix(vec4(0,0,0,1), vec4(1,1,1,0.3), fg_bg);
}")

(defgeneric debug-draw (thing &key))

(define-shader-entity debug-draw (renderable)
  ((name :initform 'debug-draw)
   (instances :accessor instances)
   (points-vao :accessor points-vao)
   (points :accessor points)
   (lines-vao :accessor lines-vao)
   (lines :accessor lines)
   (text-render :accessor text-render)))

(defmethod initialize-instance :after ((draw debug-draw) &key)
  (setf (instances draw) (make-array 64 :fill-pointer 0 :adjustable T :element-type '(unsigned-byte 32)))
  (setf (points draw) (make-array 64 :fill-pointer 0 :adjustable T :element-type 'single-float))
  (let ((vbo (make-instance 'vertex-buffer :buffer-data (points draw))))
    (setf (points-vao draw) (make-instance 'vertex-array :vertex-form :points :bindings
                                           `((,vbo :offset  0 :stride 24)
                                             (,vbo :offset 12 :stride 24)))))
  (setf (lines draw) (make-array 128 :fill-pointer 0 :adjustable T :element-type 'single-float))
  (let ((vbo (make-instance 'vertex-buffer :buffer-data (lines draw))))
    (setf (lines-vao draw) (make-instance 'vertex-array :vertex-form :lines :bindings
                                          `((,vbo :offset  0 :stride 24)
                                            (,vbo :offset 12 :stride 24)))))
  (setf (text-render draw) (make-instance 'debug-draw-text)))

(defmethod text-vao ((draw debug-draw))
  (text-vao (text-render draw)))

(defmethod text ((draw debug-draw))
  (text (text-render draw)))

(defmethod stage ((draw debug-draw) (area staging-area))
  (stage (points-vao draw) area)
  (stage (lines-vao draw) area)
  (stage (text-render draw) area))

(defmethod render ((draw debug-draw) (program shader-program))
  (setf (uniform program "view_matrix") (view-matrix))
  (setf (uniform program "projection_matrix") (projection-matrix))
  (disable-feature :depth-test)
  (render-array (points-vao draw) :vertex-count (truncate (length (points draw)) 6))
  (render-array (lines-vao draw) :vertex-count (truncate (length (lines draw)) 6))
  (enable-feature :depth-test)
  (render (text-render draw) T)
  (let ((scene (scene draw)))
    (unless (eq draw (elt scene (1- (length scene))))
      (warn "~S is not the last entity in the scene" draw)
      (leave draw T)
      (enter draw scene))))

(defun debug-draw-allocate (data instances instance type n)
  (cond (instance
         (let ((diff (- (* 3 n) (aref instances (+ 2 (* 3 instance)))))
               (i (aref instances (+ 1 (* 3 instance)))))
           (unless (= 0 diff)
             (array-utils:array-shift data :n diff :from i)
             (loop for inst from (* 3 instance) below (length instances) by 3
                   when (= type (aref instances inst))
                   do (incf (aref instances (1+ inst)) diff)))
           (values i instance)))
        (T
         (let ((i (length data)))
           (when (< (array-total-size data) (+ (length data) (* 3 n)))
             (adjust-array data (+ (length data) (* 3 n))))
           (incf (fill-pointer data) (* 3 n))
           (setf instance (truncate (length instances) 3))
           (vector-push-extend type instances)
           (vector-push-extend i instances)
           (vector-push-extend (* 3 n) instances)
           (values i instance)))))

(defmacro define-debug-draw-function ((name type) args &body body)
  (let ((type-id (ecase type (points 1) (lines 2) (text 3))))
    `(defun ,name (,@args (debug-draw (node 'debug-draw T)) (update T) (container (scene +main+)) instance)
       (flet ((,name ()
                (unless debug-draw
                  (setf debug-draw (enter-and-load (make-instance 'debug-draw) container +main+)))
                (let* ((data (,type debug-draw))
                       (instances (instances debug-draw))
                       (i 0))
                  (declare (type (vector single-float) data))
                  (declare (type (vector (unsigned-byte 32)) instances))
                  (declare (type (unsigned-byte 32) i))
                  (flet ((v (v)
                           (setf (aref data (+ 0 i)) (vx v))
                           (setf (aref data (+ 1 i)) (vy v))
                           (setf (aref data (+ 2 i)) (vz v))
                           (incf i 3))
                         (allocate (n)
                           (multiple-value-setq (i instance)
                             (debug-draw-allocate data instances instance ,type-id n))))
                    (declare (ignorable #'v))
                    ,@body))
                (when update
                  (resize-buffer (caar (bindings (,(ecase type (points 'points-vao) (lines 'lines-vao) (text 'text-vao)) debug-draw))) T))
                instance))
         (if (current-p (context +main+))
             (,name)
             (with-eval-in-render-loop (T :block T)
               (,name)))))))

(defmethod debug-draw ((point vec2) &rest args &key &allow-other-keys)
  (apply #'debug-point (vxy_ point) args))

(defmethod debug-draw ((point vec3) &rest args &key &allow-other-keys)
  (apply #'debug-point point args))

(define-debug-draw-function (debug-point points) (point &key (color #.(vec 1 0 0)))
  (allocate 2)
  (v point)
  (v color))

(define-debug-draw-function (debug-line lines) (a b &key (color #.(vec 1 0 0)) (color-a color) (color-b color))
  (allocate 4)
  (v a)
  (v color-a)
  (v b)
  (v color-b))

(defmethod debug-draw ((string string) &rest args &key &allow-other-keys)
  (let ((point (getf args :point)))
    (remf args :point)
    (apply #'debug-text (or point (vec 0 0 0)) string args)))

(define-debug-draw-function (debug-text text) (point text &key (scale 0.2))
  (allocate (* 5 2 (length text)))
  (print-ascii-text text data :start i
                              :adjust NIL
                              :x (vx point)
                              :y (vy point)
                              :z (if (typep point 'vec2) 0.0 (vz point))
                              :scale scale))

(defmethod debug-draw ((primitive box) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-box #.(vec 0 0 0) (box-bsize primitive) args))

(define-debug-draw-function (debug-box lines) (location bsize &key (color #.(vec 1 0 0)) (transform (model-matrix)))
  (allocate (* 12 2 2))
  (flet ((line (a b)
           (v (n*m transform (v+ location a))) (v color)
           (v (n*m transform (v+ location b))) (v color)))
    (let ((w (vx bsize)) (h (vy bsize)) (d (vz bsize)))
      (line (vec (- w) (- h) (- d)) (vec (+ w) (- h) (- d)))
      (line (vec (+ w) (- h) (- d)) (vec (+ w) (+ h) (- d)))
      (line (vec (+ w) (+ h) (- d)) (vec (- w) (+ h) (- d)))
      (line (vec (- w) (+ h) (- d)) (vec (- w) (- h) (- d)))

      (line (vec (- w) (- h) (+ d)) (vec (+ w) (- h) (+ d)))
      (line (vec (+ w) (- h) (+ d)) (vec (+ w) (+ h) (+ d)))
      (line (vec (+ w) (+ h) (+ d)) (vec (- w) (+ h) (+ d)))
      (line (vec (- w) (+ h) (+ d)) (vec (- w) (- h) (+ d)))

      (line (vec (- w) (- h) (- d)) (vec (- w) (- h) (+ d)))
      (line (vec (+ w) (- h) (- d)) (vec (+ w) (- h) (+ d)))
      (line (vec (+ w) (+ h) (- d)) (vec (+ w) (+ h) (+ d)))
      (line (vec (- w) (+ h) (- d)) (vec (- w) (+ h) (+ d))))))

(defmethod debug-draw ((primitive sphere) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-sphere #.(vec 0 0 0) (sphere-radius primitive) args))

(define-debug-draw-function (debug-sphere lines) (location radius &key (color #.(vec 1 0 0)) (segments 16) (transform (model-matrix)))
  (allocate (* 3 (1+ segments) 2 2))
  (labels ((line (a b)
             (v (n*m transform (nv+ a location))) (v color)
             (v (n*m transform (nv+ b location))) (v color))
           (circle (coerce)
             (loop for i from 0 to segments
                   for rad-1 = (* F-2PI (/ (+ i 0) segments))
                   for rad-2 = (* F-2PI (/ (+ i 1) segments))
                   do (line (funcall coerce (vec (* radius (cos rad-1)) (* radius (sin rad-1))))
                            (funcall coerce (vec (* radius (cos rad-2)) (* radius (sin rad-2))))))))
    (circle #'vxy_)
    (circle #'vx_y)
    (circle #'v_xy)))

(defmethod debug-draw ((primitive ellipsoid) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-ellipsoid #.(vec 0 0 0) (ellipsoid-radius primitive) args))

(define-debug-draw-function (debug-ellipsoid lines) (location radius &key (color #.(vec 1 0 0)) (segments 16) (transform (model-matrix)))
  (allocate (* 3 (1+ segments) 2 2))
  (labels ((line (a b)
             (v (n*m transform (nv+ a location))) (v color)
             (v (n*m transform (nv+ b location))) (v color))
           (circle (coerce)
             (loop for i from 0 to segments
                   for rad-1 = (* F-2PI (/ (+ i 0) segments))
                   for rad-2 = (* F-2PI (/ (+ i 1) segments))
                   do (line (funcall coerce (cos rad-1) (sin rad-1))
                            (funcall coerce (cos rad-2) (sin rad-2))))))
    (circle (lambda (a b) (vec (* (vx radius) a) (* (vy radius) b) 0)))
    (circle (lambda (a b) (vec (* (vx radius) a) 0 (* (vz radius) b))))
    (circle (lambda (a b) (vec 0 (* (vy radius) a) (* (vz radius) b))))))

(defmethod debug-draw ((primitive cylinder) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-cylinder #.(vec 0 0 0) (cylinder-radius primitive) (cylinder-height primitive) args))

(define-debug-draw-function (debug-cylinder lines) (location radius height &key (color #.(vec 1 0 0)) (segments 16) (transform (model-matrix)))
  (allocate (+ (* 2 (1+ segments) 2 2) (* 4 2 2 2)))
  (labels ((line (a b)
             (v (n*m transform (nv+ a location))) (v color)
             (v (n*m transform (nv+ b location))) (v color))
           (circle (y)
             (loop for i from 0 below segments
                   for rad-1 = (* F-2PI (/ (+ i 0) segments))
                   for rad-2 = (* F-2PI (/ (+ i 1) segments))
                   do (line (vec (* radius (cos rad-1)) y (* radius (sin rad-1)))
                            (vec (* radius (cos rad-2)) y (* radius (sin rad-2)))))))
    (circle (+ height))
    (circle (- height))
    (line (vec (- radius) (- height) 0.0) (vec (- radius) (+ height) 0.0))
    (line (vec (+ radius) (- height) 0.0) (vec (+ radius) (+ height) 0.0))
    (line (vec 0.0 (- height) (- radius)) (vec 0.0 (+ height) (- radius)))
    (line (vec 0.0 (- height) (+ radius)) (vec 0.0 (+ height) (+ radius)))))

(defmethod debug-draw ((primitive pill) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-pill #.(vec 0 0 0) (pill-radius primitive) (pill-height primitive) args))

(define-debug-draw-function (debug-pill lines) (location radius height &key (color #.(vec 1 0 0)) (segments 16) (transform (model-matrix)))
  (allocate (+ (* 2 (1+ segments) 2 2) (* 4 2 2 2) (* 2 segments 2 2)))
  (labels ((line (a b)
             (v (n*m transform (nv+ a location))) (v color)
             (v (n*m transform (nv+ b location))) (v color))
           (circle (y)
             (loop for i from 0 below segments
                   for rad-1 = (* F-2PI (/ (+ i 0) segments))
                   for rad-2 = (* F-2PI (/ (+ i 1) segments))
                   do (line (vec (* radius (cos rad-1)) y (* radius (sin rad-1)))
                            (vec (* radius (cos rad-2)) y (* radius (sin rad-2))))))
           (semicircle (y coerce start)
             (loop for i from start below (+ start (truncate segments 2))
                   for rad-1 = (* F-2PI (/ (+ i 0) segments))
                   for rad-2 = (* F-2PI (/ (+ i 1) segments))
                   do (line (nv+ (funcall coerce (vec (* radius (cos rad-1)) (* radius (sin rad-1)))) (vec 0 y 0))
                            (nv+ (funcall coerce (vec (* radius (cos rad-2)) (* radius (sin rad-2)))) (vec 0 y 0))))))
    (circle (+ height))
    (circle (- height))
    (semicircle (+ height) #'vxy_ 0)
    (semicircle (- height) #'vxy_ (* segments 2/4))
    (semicircle (+ height) #'v_xy (* segments 3/4))
    (semicircle (- height) #'v_xy (* segments 1/4))
    (line (vec (- radius) (- height) 0.0) (vec (- radius) (+ height) 0.0))
    (line (vec (+ radius) (- height) 0.0) (vec (+ radius) (+ height) 0.0))
    (line (vec 0.0 (- height) (- radius)) (vec 0.0 (+ height) (- radius)))
    (line (vec 0.0 (- height) (+ radius)) (vec 0.0 (+ height) (+ radius)))))

(defmethod debug-draw ((primitive triangle) &rest args &key &allow-other-keys)
  (let ((verts (make-array 9 :element-type 'single-float))
        (faces (make-array 3 :element-type '(unsigned-byte 32) :initial-contents '(0 1 2))))
    (declare (dynamic-extent verts faces))
    (setf (aref verts 0) (vx (triangle-a primitive)))
    (setf (aref verts 1) (vy (triangle-a primitive)))
    (setf (aref verts 2) (vz (triangle-a primitive)))
    (setf (aref verts 3) (vx (triangle-b primitive)))
    (setf (aref verts 4) (vy (triangle-b primitive)))
    (setf (aref verts 5) (vz (triangle-b primitive)))
    (setf (aref verts 6) (vx (triangle-c primitive)))
    (setf (aref verts 7) (vy (triangle-c primitive)))
    (setf (aref verts 8) (vz (triangle-c primitive)))
    (apply #'debug-triangles verts faces args)))

(defmethod debug-draw ((entity vertex-entity) &rest args &key &allow-other-keys)
  (apply #'debug-vertex-array (vertex-array entity) args))

(defmethod debug-draw :around ((entity transformed-entity) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (tmat (tf entity))))
  (apply #'call-next-method entity args))

(defmethod debug-draw ((data mesh-data) &rest args &key &allow-other-keys)
  (apply #'debug-triangles (reordered-vertex-data data '(location)) (faces data) args))

(defmethod debug-draw ((primitive general-mesh) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (apply #'debug-triangles (general-mesh-vertices primitive) (general-mesh-faces primitive) args))

(defmethod debug-draw ((primitive primitive) &rest args &key &allow-other-keys)
  (unless (getf args :transform)
    (setf (getf args :transform) (primitive-transform primitive)))
  (let ((primitive (coerce-object primitive 'convex-mesh)))
    (apply #'debug-triangles (general-mesh-vertices primitive) (general-mesh-faces primitive) args)))

(define-debug-draw-function (debug-triangles lines) (vertices faces &key (color #.(vec 1 0 0)) (transform (model-matrix)))
  (allocate (* (length faces) 2 2))
  (flet ((lines (vec)
           (v vec)
           (v color)))
    (loop with prev
          for e across faces
          for c = 0 then (mod (1+ c) 3)
          for i = (* e 3)
          do (let ((vec (vec (aref vertices (+ 0 i)) (aref vertices (+ 1 i)) (aref vertices (+ 2 i)) 1.0)))
               (declare (dynamic-extent vec))
               (n*m transform vec)
               (case c
                 (0 (lines vec)
                  (setf prev vec))
                 (1 (lines vec)
                  (lines vec))
                 (T (lines vec)
                  (lines prev)
                  (lines vec)))))))

(define-debug-draw-function (debug-vertex-array lines) (vao &key (color #.(vec 1 0 0)) (transform (model-matrix)))
  (let ((count 0) prev pprev)
    (labels ((lines (vec)
               (v vec)
               (v color))
             (line-strip (vec)
               (case count
                 (0 (lines vec)
                  (setf count 1))
                 (1 (lines vec)
                  (lines vec))))
             (line-loop (vec)
               (declare (ignore vec))
               (implement!))
             (triangles (vec)
               (case count
                 (0 (lines vec)
                  (setf prev vec)
                  (setf count 1))
                 (1 (lines vec)
                  (lines vec)
                  (setf count 2))
                 (T (lines vec)
                  (lines prev)
                  (lines vec)
                  (setf count 0))))
             (triangle-strip (vec)
               (case count
                 (0 (lines vec)
                  (setf pprev vec)
                  (setf count 1))
                 (1 (lines vec)
                  (setf prev vec)
                  (setf count 2))
                 (T (lines vec)
                  (lines prev)
                  (lines vec)
                  (lines pprev)
                  (shiftf pprev prev vec))))
             (triangle-fan (vec)
               (case count
                 (0 (lines prev)
                  (setf pprev vec)
                  (setf count 1))
                 (1 (lines vec)
                  (setf prev vec)
                  (setf count 2))
                 (T (lines vec)
                  (lines prev)
                  (lines vec)
                  (lines pprev)
                  (setf prev vec))))
             (alloc (verts)
               (ecase (vertex-form vao)
                 (:lines (allocate (* 2 verts)))
                 (:line-strip (allocate (* 2 2 (1- verts))))
                 (:line-loop (allocate (* 2 2 verts)))
                 (:triangles (allocate (* 2 2 verts)))
                 (:triangle-strip (allocate (* 2 2 (1+ (* 2 (- verts 2))))))
                 (:triangle-fan (allocate (* 2 2 (1+ (* 2 (- verts 2)))))))))
      (let ((vertex (ecase (vertex-form vao)
                      (:lines #'lines)
                      (:line-strip #'line-strip)
                      (:line-loop #'line-loop)
                      (:triangles #'triangles)
                      (:triangle-strip #'triangle-strip)
                      (:triangle-fan #'triangle-fan)))
            ebo vbo)
        (loop for binding in (bindings vao)
              do (case (buffer-type (unlist binding))
                   (:element-array-buffer
                    (setf ebo (buffer-data (unlist binding))))
                   (:array-buffer
                    (when (= 0 (getf (rest binding) :index 0))
                      (setf vbo binding)))))
        (destructuring-bind (buffer &key (size 3) (stride 0) (offset 0) &allow-other-keys) vbo
          (let ((data (buffer-data buffer)))
            (cond (ebo
                   (alloc (length ebo))
                   (loop for e across ebo
                         for i = (+ (floor offset 4) (* (floor stride 4) e))
                         do (ecase size
                              (3
                               (let ((vec (vec (aref data (+ 0 i)) (aref data (+ 1 i)) (aref data (+ 2 i)) 1.0)))
                                 (declare (dynamic-extent vec))
                                 (n*m transform vec)
                                 (funcall vertex vec)))
                              (2
                               (let ((vec (vec (aref data (+ 0 i)) (aref data (+ 1 i)) 0.0 1.0)))
                                 (declare (dynamic-extent vec))
                                 (n*m transform vec)
                                 (funcall vertex vec))))))
                  (T
                   (alloc (truncate (- (length data) (floor offset 4)) (floor stride 4)))
                   (loop for i from (floor offset 4) by (floor stride 4) below (length data)
                         do (ecase size
                              (3
                               (let ((vec (vec (aref data (+ 0 i)) (aref data (+ 1 i)) (aref data (+ 2 i)) 1.0)))
                                 (declare (dynamic-extent vec))
                                 (n*m transform vec)
                                 (funcall vertex vec)))
                              (2
                               (let ((vec (vec (aref data (+ 0 i)) (aref data (+ 1 i)) 0.0 1.0)))
                                 (declare (dynamic-extent vec))
                                 (n*m transform vec)
                                 (funcall vertex vec)))))))))))))

;; TODO: these
#++(define-debug-draw-function (debug-vector lines) (location direction &key (color #.(vec 1 0 0)))
  )

(defun debug-clear (&key (debug-draw (node 'debug-draw T)) (update T) instance)
  (when debug-draw
    (cond (instance
           (let* ((type (aref (instances debug-draw) (+ 0 (* 3 instance))))
                  (start (aref (instances debug-draw) (+ 1 (* 3 instance))))
                  (size (aref (instances debug-draw) (+ 2 (* 3 instance))))
                  (data (ecase type
                          (1 (points debug-draw))
                          (2 (lines debug-draw))
                          (3 (text debug-draw)))))
             (array-utils:array-shift data :n size :from start)
             (when (< (* 3 (1+ instance)) (length (instances debug-draw)))
               (array-utils:array-shift (instances debug-draw) :n -3 :from (* 3 (1+ instance))))))
          (T
           (setf (fill-pointer (points debug-draw)) 0)
           (setf (fill-pointer (lines debug-draw)) 0)
           (setf (fill-pointer (text debug-draw)) 0)
           (setf (fill-pointer (instances debug-draw)) 0)))
    (when update
      (resize-buffer (caar (bindings (points-vao debug-draw))) T)
      (resize-buffer (caar (bindings (lines-vao debug-draw))) T)
      (resize-buffer (caar (bindings (text-vao debug-draw))) T))))

(define-class-shader (debug-draw :vertex-shader)
  "layout (location = 0) in vec3 position;
layout (location = 1) in vec3 i_color;
out vec3 v_color;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main(){
  gl_Position = projection_matrix * view_matrix * vec4(position, 1.0f);
  v_color = i_color;
}")

(define-class-shader (debug-draw :fragment-shader)
  "in vec3 v_color;
out vec4 color;

void main(){
  color = vec4(v_color, 1);
}")
