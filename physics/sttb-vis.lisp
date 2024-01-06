#++
(ql:quickload '(3b-glim 3b-glim-ui trial-examples))
(defpackage #:sttb-test
  (:use #:cl)
  (:local-nicknames (#:a #:alexandria-2)
                    (#:u #:3b-glim-ui)
                    (#:glim #:3b-glim/s)
                    (#:s #:org.shirakumo.fraf.trial.sttb)
                    (#:trial #:org.shirakumo.fraf.trial)
                    (#:m #:org.shirakumo.fraf.math)))
(in-package #:sttb-test)

(defclass foo (u:window)
  ((flags :initform (make-hash-table) :reader flags)
   (c :initform 0 :accessor c)
   (a :initform 0 :accessor a)
   (a2 :initform 0 :accessor a2)
   (then :initform 0 :accessor then)))

(defmethod flag ((w foo) x)
  (gethash x (flags w)))

(defmethod (setf flag) (n (w foo) x)
  (setf (gethash x (flags w)) n))

(defvar *w* nil)
(defvar *rs* (make-random-state))
(defmethod u:display ((w foo) now)
  (setf *w* w)
  ;; draw to screen behind UI
)

(defparameter *debug-state* (make-hash-table))
(defparameter *steps* nil)
(defvar *step* 0)
(defparameter *ref* nil)
(defparameter *gjk* nil)
(defparameter *ref-hit* nil)
(defparameter *gjk-hit* nil)
(defparameter *test-idx* 0)
(defparameter *notes* nil)
(defvar *result* nil)
(defvar *done* nil)
(defvar *pushed* nil)
(defvar *pushed2* nil)
(defvar *pushed3* nil)
(defvar *tests* (list (list :box (m:vec3 1.5 0.25 0.125) (m:quat)
                            :box (m:vec3 0 0 0) (m:quat))
                      (list :sphere (m:vec3 1.5 0.25 0.125) (m:quat)
                            :box (m:vec3 0 0 0) (m:quat))
                      (list :cylinder (m:vec3 1.5 0.25 0.125) (m:quat)
                            :box (m:vec3 0 0 0) (m:quat))
                      (list :sphere (m:vec3 1.5 0.25 0.125) (m:quat)
                            :pill (m:vec3 0 0 0) (m:quat))
                      (list
                       :PILL (m:VEC3 0.70053387 0.11000085 0.56530666)
                       (m:QUAT 0.2644229 -0.52753115 0.27729535 0.75822073)
                       :SPHERE (m:VEC3 -0.91779256 0.31338882 0.11587548)
                       (m:QUAT -0.6981535 -0.18972588 0.68339735 0.09774389))
                      (list
                       :PILL (m:VEC3 -1.0854028 -1.3133026 -0.21314728)
                       (m:QUAT -0.5478256 -0.75387454 -0.1941125 -0.3063995)
                       :PILL (m:VEC3 0.9302671 0.49484396 -0.6091559)
                       (m:QUAT 0.7429185 0.572958 -0.17468882 0.29878935))))

#++
(list *shape1* *pos1* *quat1*
      *shape2* *pos2* *quat2*
      )
(defvar *tests/cm*      ;; convex mesh tests
  (list (list :CYLINDER/cm ;; no intersection, ~4 points cycle
              (m:VEC3 -0.48472548 -0.25022292 0.34735203)
              (m:QUAT 0.41110802 0.30641174 0.7947355 -0.32480383)
              :CYLINDER/cm
              (m:VEC3 0.9476633 -0.8821726 0.5045867)
              (m:QUAT 0.49228263 -0.21329689 -0.33321455 -0.7753259))
        (list :CYLINDER/cm ;; no intersection, 2 point cycle
              (m:VEC3 -0.03711605 0.823828 -0.18367815)
              (m:QUAT -0.49869204 0.3446517 0.77065617 0.19649573)
              :SPHERE/CM
              (m:VEC3 -0.6897297 -0.94591594 -0.014652729)
              (m:QUAT 0.073975325 0.8018396 0.48449442 0.34182754))
        (list :SPHERE/CM
              (m:VEC3 0.9995713 0.8681288 -0.7726667)
              (m:QUAT 0.28125334 0.25453806 0.68861514 -0.61799365)
              :PILL/CM
              (m:VEC3 -0.4301839 -0.6971922 0.61376286)
              (m:QUAT -0.10118961 0.3455479 -0.482602 0.7984063))
        (list :PILL/CM (m:VEC3 0.7378607 -0.86003613 0.8793256)
              (m:QUAT -0.64264846 -0.29890704 0.34350827 0.6161653)
              :BOX/CM (m:VEC3 0.7482915 0.29556823 0.2796347)
              (m:QUAT -0.6831958 -0.49483132 0.20551045 0.49613592))))

(defvar *tests2*
  (list (list :box #(0.9130889 0.0 0.40776047 -1.3199992
                     0.0 1.0 0.0 0.0 -0.40776047 0.0
                     0.9130889 -0.34999824 0.0 0.0 0.0 1.0)
              :ellipsoid #(0.41759455 0.21598464 -0.88259023 -0.23999998
                           7.450581e-9 0.971338 0.23770273 0.0
                           0.90863353 -0.09926337 0.4056254 0.39000002
                           0.0 0.0 0.0 1.0
                           ))
        (list :box #(1.0 0.0 0.0 -0.14999998
                     0.0 1.0 0.0 -0.10999817
                     0.0 0.0 1.0 0.0
                     0.0 0.0 0.0 1.0)
              :ellipsoid #(1.0 0.0 0.0 0.0
                           0.0 1.0 0.0 0.0
                           0.0 0.0 1.0 0.0
                           0.0 0.0 0.0 1.0))))


(defvar *shape1* :box)
(defvar *shape2* :pill)
(defvar *pos1* (m:vec3 0.5 0 0))
(defvar *pos2* (m:vec3 0 0 0))
(defvar *quat1* (m:quat))
(defvar *quat2* (m:quat))
(defvar *mat1* nil)
(defvar *mat2* nil)
(defvar *obj1* ())
(defvar *obj2* ())
(defvar *mesh1* ())
(defvar *mesh2* ())
#++
(setf (values *shape1* *pos1* *quat1*
              *shape2* *pos2* *quat2*
              *mat1* *mat2*)
      (values-list (nth 5 *tests*)))


#++
(setf (values *shape1* *mat1* *shape2* *mat2*)
      (values-list (nth 1 *tests2*)))


(defvar *obj-orientation* (m:quat))

(defun %run (shape1 loc1 quat1 shape2 loc2 quat2 mat1 mat2)
  (flet ((shape (shape)
           (let ((s (ecase shape
                      ((:box :box/cm) (trial:make-box))
                      ((:sphere :sphere/cm) (trial:make-sphere))
                      ((:cylinder :cylinder/cm) (trial:make-cylinder))
                      ((:pill :pill/cm) (trial:make-pill))
                      ((:ellipsoid :ellipsoid/cm)
                       (trial:make-ellipsoid :radius (m:nvunit (m:vec3 3 2 1)))))))
             (when (a:ends-with-subseq "/CM" (string shape))
               (setf s (trial:coerce-object s 'trial:convex-mesh)))
             s))
         (xform (obj loc quat mat)
           (cond
             (mat
              (replace (m:marr (trial:global-transform-matrix obj)) mat))
             (t (setf (trial:location obj) loc)
                (setf (trial:orientation obj) quat)
                (m:m<- (trial:global-transform-matrix obj)
                       (trial:primitive-local-transform obj))))))
    (let* ((a (setf *obj1* (shape shape1)))
           (b (setf *obj2* (shape shape2)))
           (hit (trial::make-hit)))
      (xform a loc1 quat1 mat1)
      (xform b loc2 quat2 mat2)

      (let ((r (org.shirakumo.fraf.trial.sttb::%sttb a b hit)))
       (values r hit a b)))))


(defun run ()
  (let ((ref-hit nil)
        (gjk-hit nil))
    (multiple-value-bind (hitp hit1 a b)
        (let ((org.shirakumo.fraf.trial.sttb::*trace* t))
          (multiple-value-prog1
              (%run *shape1* *pos1* *quat1* *shape2* *pos2* *quat2* *mat1* *mat2*)
            (setf *steps* org.shirakumo.fraf.trial.sttb::*trace*)))
      (declare (ignorable hitp hit1))
      (let ((hits (vector (trial:make-hit))))
        (setf *ref* "no hit")
        (setf *ref-hit* nil)
        (when (= 1 (org.shirakumo.fraf.trial.mpr::detect-hits a b hits 0 1))
          (setf ref-hit (aref hits 0))
          (setf *ref-hit* (aref hits 0))
          (setf *ref* (format nil "mpr depth ~s~% normal ~a~% loc ~a"
                              (trial:hit-depth (aref hits 0))
                              (trial:hit-normal (aref hits 0))
                              (trial:hit-location (aref hits 0))))))
      (let ((hits (vector (trial:make-hit))))
        (setf *gjk* "no hit")
        (setf *gjk-hit* nil)
        (when (= 1 (org.shirakumo.fraf.trial.gjk::detect-hits a b hits 0 1))
          (setf gjk-hit (aref hits 0))
          (setf *gjk-hit* (aref hits 0))
          (setf *gjk* (format nil "gjk depth ~s~% normal ~a~% loc ~a"
                              (trial:hit-depth (aref hits 0))
                              (trial:hit-normal (aref hits 0))
                              (trial:hit-location (aref hits 0))))))
      (setf *pushed* nil)
      (setf *pushed2* nil)
      (setf *pushed3* nil)
      (when (and (not *mat1*) hitp)
        (multiple-value-bind (hitp2 hit2 a b)
            (%run *shape1* (m:v+ *pos1*
                                 (m:v* (trial::hit-normal hit1)
                                       (trial::hit-depth hit1)))
                  *quat1*
                  *shape2* *pos2* *quat2* nil nil)
          (let* ((hits (vector (trial:make-hit)))
                 (r (org.shirakumo.fraf.trial.mpr::detect-hits a b hits 0 1))
                 (ghits (vector (trial:make-hit)))
                 (gr (org.shirakumo.fraf.trial.gjk::detect-hits a b hits 0 1)))
            (setf *pushed*
                  (format nil "pushed hit=~s depth ~s~% mpr ~s, depth ~s~% gjk ~s, depth ~s~%"
                          hitp2 (trial:hit-depth hit2)
                          r (unless (zerop r) (trial:hit-depth (aref hits 0)))
                          gr (unless (zerop gr) (trial:hit-depth (aref ghits 0))))))))
      (when (and (not *mat1*) ref-hit)
        (multiple-value-bind (hitp2 hit2 a b)
            (%run *shape1* (m:v+ *pos1*
                                 (m:v* (trial::hit-normal ref-hit)
                                       (trial::hit-depth ref-hit)))
                  *quat1*
                  *shape2* *pos2* *quat2* nil nil)
          (let* ((hits (vector (trial:make-hit)))
                 (r (org.shirakumo.fraf.trial.mpr::detect-hits a b hits 0 1))
                 (ghits (vector (trial:make-hit)))
                 (gr (org.shirakumo.fraf.trial.gjk::detect-hits a b hits 0 1)))
            (setf *pushed2*
                  (format nil "pushed2 hit=~s depth ~s~% mpr ~s, depth ~s~% gjk ~s, depth ~s~%"
                          hitp2 (trial:hit-depth hit2)
                          r (unless (zerop r) (trial:hit-depth (aref hits 0)))
                          gr (unless (zerop gr) (trial:hit-depth (aref ghits 0))))))))
      (when (and (not *mat1*) ref-hit)
        (multiple-value-bind (hitp2 hit2 a b)
            (%run *shape1* (m:v+ *pos1*
                                 (m:v* (trial::hit-normal gjk-hit)
                                       (trial::hit-depth gjk-hit)))
                  *quat1*
                  *shape2* *pos2* *quat2* nil nil)
          (let* ((hits (vector (trial:make-hit)))
                 (r (org.shirakumo.fraf.trial.mpr::detect-hits a b hits 0 1))
                 (ghits (vector (trial:make-hit)))
                 (gr (org.shirakumo.fraf.trial.gjk::detect-hits a b hits 0 1)))
            (setf *pushed3*
                  (format nil "pushed3 hit=~s depth ~s~% mpr ~s, depth ~s~% gjk ~s, depth ~s~%"
                          hitp2 (trial:hit-depth hit2)
                          r (unless (zerop r) (trial:hit-depth (aref hits 0)))
                          gr (unless (zerop gr) (trial:hit-depth (aref ghits 0))))))))))



  (setf *done* nil)
  (setf *step* 0))


(defparameter *zoom* 380)


(defvar *auto* nil)
(defvar *auto-state* :done)
(defvar *auto-time* 0)
(defvar *fast-time* 0.01)
(defvar *step-time* 0.01)
(defvar *auto-errors* nil)
(defvar *old-errors* nil)
#++
(setf *old-errors* (append *auto-errors* *old-errors*))
(defvar *auto-log* nil)
(defvar *auto-max-err* 0)
(defvar *auto-count* 0)
(defvar *auto-hits* 0)
(defvar *auto-+ref* 0)
(defvar *auto-+hit* 0)
(defvar *auto-misses* 0)
(defvar *auto-distances* ())

(defvar *auto-random* (make-random-state))
(defvar *random-queue* nil)
;(setf *random-queue* *old-errors*)

(defun random-xy<1 ()
  (flet ((r () (- (random 2.0) 1.0)))
   (loop for x = (r)
         for y = (r)
         for x2 = (expt x 2)
         for y2 = (expt y 2)
         for z = (+ x2 y2)
         while (<= 1 z)
         finally (return (values x y z x2 y2)))))
#++(random-point-on-sphere (radius)
               (multiple-value-bind (x y z x2 y2) (random-xy<1)
                 (let ((r (sqrt (- 1 x2 y2))))
                   (v* (vec3 (* 2 x r)
                             (* 2 y r)
                             (- 1 (* 2 z)))
                       radius))))
(defun random-orientation ()
  (multiple-value-bind (x y z) (random-xy<1)
    (multiple-value-bind (u v w) (random-xy<1)
      (let ((s (sqrt (/ (- 1 z) w))))
        (m:quat x y (* s u) (* s v))))))

(defun random-shape ()
  (ecase (random 10)
    (0 :sphere)
    (1 :sphere/cm)
    (2 :box)
    (3 :box/cm)
    (4 :cylinder)
    (5 :cylinder)
    (6 :pill)
    (7 :pill)
    (8 :ellipsoid)
    (9 :ellipsoid/cm)))

(defun random-test ()
  (setf *shape1* (random-shape); :pill
        )
  (setf *shape2* (random-shape))
  (setf *pos1* (m:vec (- (random 2.0) 1.0)
                      (- (random 2.0) 1.0)
                      (- (random 2.0) 1.0)))
  (setf *pos2* (m:vec (- (random 2.0) 1.0)
                      (- (random 2.0) 1.0)
                      (- (random 2.0) 1.0)))
  (setf *quat1* (random-orientation))
  (setf *quat2* (random-orientation))
  (setf *mat1* nil *mat2* nil))
  
#++
(defun write-tests ()
  (with-open-file (f (asdf:system-relative-pathname
                      'trial "physics/3b-gjk-regression3.lisp")
                     :if-does-not-exist :create
                     :if-exists :append
                     :direction :output)
    
    (loop for (a b c . r) in *auto-errors*
          do (format f "(~a~% ~a~% ~s" a b c)
             (loop for (k v) on r by #'cddr
                   do (format f "~% ~s " k)
                      (if (typep v 'condition)
                          (format f "\"~a\"" v)
                          (format f "~a" v)))
             (format f ")~%")
          )))
#++
(defun auto-test (now)
  (labels ((dt ()
             (- now *auto-time*))
           (add-case (&rest r &key ref gjk d steps)
             (declare (ignore ref gjk d steps))
             (push (list* *ray-start* *ray-dir* *test-obj* r)
                   *auto-errors*))
           (l (&rest r)
             (push (apply #'format nil r) *auto-log*))
           (run1 (steps)
             (setf *auto-time* now)
             (let* ((s::*steps* steps)
                    (ray (trial:ray *ray-start* *ray-dir*))
                    (mesh (setf *obj* (trial:coerce-object
                                       (a:eswitch (*test-obj*)
                                         (nil (trial:make-box))
                                         (:box (trial:make-box))
                                         (:sphere (trial:make-sphere))
                                         (:cylinder (trial:make-cylinder)))
                                       'trial:convex-mesh)))
                    (obj (make-instance 'trial:rigidbody
                                        :name :foo
                                        :physics-primitives (vector mesh)
                                        )))
               (let ((*package* (find-package '#:m)))
                 #++(format t "~s/~s:~s @ ~s~%" *auto-state* steps
                            *test-obj* *ray-start*))
               (let ((s::*f* nil))
                 (multiple-value-bind (r e)
                     (ignore-errors
                      (s::ref *ray-start* *ray-dir*
                              (aref (trial:physics-primitives obj) 0)))
                   (when (typep e 'condition)
                     (add-case :ref e)
                     (setf *auto-state* :done)
                     (l "ref failed:~% ~a" e)
                     (return-from run1 nil))
                   (setf (values *ref* *ref-normal*) (values r e))))
               (multiple-value-bind (r e) (catch :step
                                            (ignore-errors (trial:detect-hit ray obj)))
                 (cond
                   ((typep e 'condition)
                    (add-case :gjk e)
                    (l "gjk failed:~% ~a" e)
                    (setf *auto-state* :done)
                    (return-from run1 nil))
                   ((eql r :step)
                    ;; do nothing
                    )
                   (t
                    (ecase *auto-state*
                      (:try
                       (if (> (gethash :steps s::*debug-state* 0) 2)
                           (setf *auto-state* :step)
                           (setf *auto-state* :start)))
                      (:step
                       (setf *auto-state* :done)))
                    (unless (eql *auto-state* :step)
                      (when (and *ref* (not r))
                        (incf *auto-+ref*))
                      (when (and r (not *ref*))
                        (incf *auto-+hit*))
                      (if r
                          (incf *auto-hits*)
                          (incf *auto-misses*))
                      (let* ((hit (when r (trial:hit-location r)))
                             (d (when (and hit *ref*) (m:vdistance hit *ref*)))
                             (*package* (find-package '#:m))
                             (fsteps (gethash :steps s::*debug-state* 0)))
                        (when d
                          (push d *auto-distances*)
                          (setf *auto-max-err* (max *auto-max-err* d)))
                        (cond
                          ((or (a:xor *ref* hit)
                               (and d (> d 0.001))
                               (> fsteps 60))
                           (l "mismatch? ~s~%  ref ~s~%  gjk ~s"
                              d *ref* hit)
                           (add-case :ref *ref* :gjk hit :steps fsteps :d d)
                           (setf *auto-state* :done))
                          (t
                           (when d
                             (setf *auto-max-err* (max *auto-max-err* d)))
                           (cond
                             ((not d)
                              #++(l "OK: miss"))
                             ((> d 0.0001)
                              (l "OK? steps:~s, d=~s~%@ ~s" fsteps d hit))
                             (t (l "OK: steps: ~s, d = ~s~% @ ~s"
                                   fsteps d hit)))
                           (setf *result*
                                 (format nil " ~s (~s/+~s,-~s/~s): max err = ~s"
                                         (incf *auto-count*)
                                         *auto-hits*
                                         *auto-+hit* *auto-+ref*
                                         *auto-misses*
                                         *auto-max-err*)))))))))))
           (try1 ()
             (setf *auto-state* :try)
             (random-test)
             (run1 nil)))
    (ecase *auto-state*
      (:start
       (setf *steps* 0)
       (when (> (dt) *fast-time*)
         (try1)))
      (:try
       (run1 nil))
      (:step
       (when (> (dt) *step-time*)
         (incf *steps*)
         (run1 *steps*)))
      (:wait)
      (:done
       (when (> (dt) (* 2 *step-time*))
         (setf *auto-state* :start)
                                        ;(setf *auto-state* :wait)
         ))))
  )

(defun draw-state (s)
  (destructuring-bind (v f) (s::mdiff s)
    (if (s::hitp s)
        (u:color 1 0 0 1)
        (u:color 0 1 0 1))
    (glim:with-draw (:lines :shader 'u:solid)
      (flet ((l (a b)
               (u:vertex (aref v (+ 0 (* a 3)))
                         (aref v (+ 1 (* a 3)))
                         (aref v (+ 2 (* a 3))))
               (u:vertex (aref v (+ 0 (* b 3)))
                         (aref v (+ 1 (* b 3)))
                         (aref v (+ 2 (* b 3))))))
        (loop for i below (length f) by 3
              for a = (aref f (+ 0 i))
              for b = (aref f (+ 1 i))
              for c = (aref f (+ 2 i))
              do (l a b)
                 (l b c)
                 (l c a))))
    (let ((step (when (< -1 *step* (length (s::steps *steps*)))
                  (aref (s::steps *steps*) *step*))))
      (when step
        (unless (a:emptyp (s::points step))
          (glim:with-draw (:points :shader 'u:solid)
            (loop ;for c from 1
                  for cc in '(#(1 0 0) #(0 1 0) #(0 0 1)
                              #(1 0 1) #(1 1 0) #(0 1 1)
                              )
                  for i across (s::points step)
                  do #++(u:color (ldb (byte 1 0) c)
                              (ldb (byte 1 1) c)
                              (ldb (byte 1 2) c))
                     (u:color (aref cc 0) (aref cc 1) (aref cc 2))
                     (u:vertex-v i))))
        (unless (a:emptyp (s::vectors step))
          (u:color 1 1 0 1)
          (glim:with-draw (:lines :shader 'u:solid)
            (loop for c from 1
                  for i across (s::vectors step)
                  do (u:color (ldb (byte 1 0) c)
                              (ldb (byte 1 1) c)
                              (ldb (byte 1 2) c))
                     (u:vertex-v i))))
        (u:color 1 1 1)
        (glim:line-width 3)
        (destructuring-bind (&optional a b c) (s::simplex step)
          (cond
            (c
             (glim:with-draw (:triangles :shader 'u:solid)
               (u:vertex-v a)
               (u:vertex-v b)
               (u:vertex-v c))
             (u:color 0 1 0 1)
             (glim:with-draw (:lines :shader 'u:solid)
               (u:color 1 0.15 0.15 1)
               (u:vertex-v a)
               (u:color 0.15 1 0.15 1)
               (u:vertex-v b)
               (u:color 0.15 0.15 1 1)
               (u:vertex-v c)
               (u:color 1 0.15 0.15 1)
               (u:vertex-v a)
               ))
            (b
             (glim:with-draw (:lines :shader 'u:solid)
               (u:vertex-v a)
               (u:vertex-v b)))
            (a
             (glim:with-draw (:points :shader 'u:solid)
               (u:vertex-v a)))))
        (glim:line-width 1)))))

(defun draw (window &optional (now 0))
  #++(when *auto* (auto-test now))
  ;; draw within a window in UI
  (if (flag window :spin)
      (incf (a2 window) (* (flag window :spin) (- now (then window))))
      (setf (flag window :spin) 0))
  (setf (then window) now)
  (u:canvas (:clear t :pixels t)
    (u::interact)
    (when (u::hot u::*context*)
      (incf (a window) (u::wheel u::*ui*)))
    (u::with-clip-dimensions (wx wy)
      (glim:with-pushed-matrix (:projection)
        (glim:load-identity)
        (glim:perspective 60 (/ wx wy) 0.8 1000)
        (glim:uniform 'u:proj (glim:ensure-matrix :projection))
        (glim:with-pushed-matrix (:modelview)
          (glim:enable :depth-test)
          (glim:load-identity)
          (let ((a (float (* (a window) (/ pi 180)) 1.0)))
            (glim:look-at (sb-cga:vec (* -10.0 (cos a)) 0.0 (* 10 (sin a)))
                          (sb-cga:vec 0.0 0.0 0.0)
                          (sb-cga:vec 0.0 0.0 1.0) ))
          (let (            ;(*random-state* (make-random-state *rs*))
                (w (* (min wx wy) 0.95))
                (z -0.01)
                )
            ;;(glim:translate (/ wx 2) (/ wy 2) 0)
            ;;(glim:scale 1 1 (/ w))
            (glim:scale 0.5 0.5 0.5)
            (glim:rotate (- (mod (* 5 (a2 window)) 360)) 0 0 1)
            (glim:uniform 'u:modelview (glim:ensure-matrix :modelview))
            (glim:point-size 10)


            (u:dispatch-draws window)
            (when *steps*
              (glim:with-pushed-matrix (:modelview)
                (glim:scale *zoom* *zoom* *zoom*)
                (glim:uniform 'u:modelview (glim:ensure-matrix :modelview))
                (draw-state *steps*)
                                        ;(glim:uniform 'u:modelview (glim:ensure-matrix :modelview))

                (let* ((n 5)
                       (w (* w 0.6))
                       (-w (- w))
                       (s (/ w n)))
                  (glim:with-draw (:quads :shader 'u:solid)
                    (u:color 0.1 0.05 0.2 0.6)
                    (u:vertex -w -w z)
                    (u:vertex -w w z)
                    (u:vertex w w z)
                    (u:vertex w -w z)
                                        ;(u:vertex -w -w)
                    )
                  (glim:with-draw (:lines :shader 'u::solid)
                    (u:color 1 0 0 1)
                    (loop for i from (- n) to n
                          do (u:vertex (* i s) (- w) z)
                             (u:vertex (* i s) w z)
                             (u:vertex (- w) (* i s) z)
                             (u:vertex w (* i s) z))
                    (let ((w 0.4))
                      (loop for i from -1 to 1 by 1/10
                            do (u:vertex (* i s) (- w) z)
                               (u:vertex (* i s) w z)
                               (u:vertex (- w) (* i s) z)
                               (u:vertex w (* i s) z)))
                    (let ((w 0.1))
                      (loop for i from -1/10 to 1/10 by 1/100
                            do (u:vertex (* i s) (- w) z)
                               (u:vertex (* i s) w z)
                               (u:vertex (- w) (* i s) z)
                               (u:vertex w (* i s) z)))
                    (when (>= *zoom* 10)
                      (let ((w 0.05))
                        (loop for i from -1/100 to 1/100 by 1/1000
                              do (u:vertex (* i s) (- w) z)
                                 (u:vertex (* i s) w z)
                                 (u:vertex (- w) (* i s) z)
                                 (u:vertex w (* i s) z))))))))))))))

(defparameter *then* 0)
(defparameter *slow* nil)

(defmethod u:draw-ui ((w foo) now)
  (u:row
    (let ((u:*text-scale* 0.6)
          ;;(*random-state* (make-random-state *random-state*))
          )
      (let ((u:*theme* u::*dark-theme*))
        (u:window (:wx 450 :clip t)
          (u:column
            (when (u:button (format nil (if *slow* "slowed!" "slow?")))
              (setf *slow* (not *slow*)))
            (when *slow* (sleep 0.03))
            (u:text (format nil "~a" (aref u::*fps* 0)))
            (u:text (format nil "~a" (aref u::*fps* 1)))
            (u:text (format nil "~a" (aref u::*fps* 2)))
            (u:row
              (u:text "a:")
              (flet ((f (n)
                       (when (u:button (format nil "~a~a"
                                               (if (flag w n) "*" " ")
                                               n))
                         (setf (flag w n) (not (flag w n))))))
                (f :v1) (f :v2) (f :x1) (f :x2)
                (f :vr) (f :vw))
              (when (u:button (format nil "~aSPIN" (case (flag w :spin)
                                                     (-1 "-") (0 "") (1 "+"))))
                (incf (flag w :spin) (or (flag w :dspin) 1))
                (unless (zerop (flag w :spin))
                  (setf (flag w :dspin) (- (flag w :spin)))))
              (u:divider :pad 10)
              (u:text "..."))
            (flet ((f (n)
                     (when (u:button (format nil "~a~a"
                                             (if (eql n *zoom*)
                                                 "*" "")
                                             n))
                       (setf *zoom* n))))
              (u:row (u:text "zoom:")
                (f 1) (f 2) (f 5) (f 10) (f 20) (f 50) (f 100) (f 200) (f 400))
              (u:row (u:text "          ") (f 1000) (f 3000) (f 10000) (f 50000)))
            (u:row (u:text "angle:")
              (flet ((f (n)
                       (when (u:button (format nil "~a~a"
                                               (if (eql n (a w))
                                                   "*" "")
                                               n))
                         (setf (a w) n))))
                (f 90) (f 75) (f 60) (f 45) (f 30) (f 15) (f 0)))
            (u:row (u:text "run random tests:")
              (when (u:button (if *auto* "stop" "start"))
                (setf *auto-state* :start)
                (setf *auto* (not *auto*)))
              (u:text (format nil "~s" *auto-state*)))
            (u:divider )
            (flet ((r () (setf *step* 0)))
              (u:row
                #++
                (when (u:button "random")
                  (setf *notes* nil)
                  #++(random-test)
                  (r)
                  (run)
                  (let ((*package* (find-package '#:m)))
                    (format t "@ ~s~%  ~s~%" *ray-start* *ray-dir*)))
                (when (u:button (format nil "<"))
                  (setf *done* nil)
                  (setf *step* (max 0 (1- *step*)))
                  #++(run))
                (when (u:button (format nil "step (~a/~a)"
                                        (if *done* "*" *step*)
                                        (when *steps*
                                          (length (s::steps *steps*)))))
                  (unless *done*
                    (incf *step*))
                  (when (and *steps*
                             (>= *step* (length (s::steps *steps*))))
                    (setf *done* t)))
                (when (u:button (format nil "redo (~a)" *step*))
                  (run))
                (when (u:button "restart")
                  (r)
                  (run))
                (when (u:button (format nil "random "))
                  (random-test)
                  (run))
                #++
                (when (u:button (format nil "next ~s/~s"
                                        (when *auto-errors*
                                          (mod *test-idx* (length *auto-errors*)))
                                        (length *auto-errors*)))
                  (destructuring-bind (rs rd &optional p &rest keys)
                      (nth (mod *test-idx* (length *auto-errors*))
                           *auto-errors*)
                    (declare (ignorable keys))
                    (setf *notes*
                          (format nil "~{~s ~a~^~%~}"
                                  keys))
                    (incf *test-idx*)
                    (setf *ray-start* rs
                          *ray-dir* rd
                          *test-obj* (or p :sphere)))
                  (r)
                  (run *steps*))
                (when (u:button "0") (setf *test-idx* 0))))
            (u:row
              (when (and *steps* (u:button "push") (not *mat1*))
                (setf *pos1* (m:v+ *pos1*
                                   (m:v* (s::normal *steps*)
                                         (s::depth *steps*)))))
              (when (and *steps* (u:button "push/m") (not *mat1*))
                (setf *pos1* (m:v+ *pos1*
                                   (m:v* (trial:hit-normal *ref-hit*)
                                         (trial:hit-depth *ref-hit*)))))
              (when (and *steps* (u:button "push/r") (not *mat1*))
                (setf *pos1* (m:v+ *pos1*
                                   (m:v* (trial:hit-normal *gjk-hit*)
                                         (trial:hit-depth *gjk-hit*))))))
            (u:divider)
            (let ((u::*text-scale* 0.5))
              (u:text (format nil "~s x ~s" *shape1* *shape2*))
              (when (and *steps* (typep *steps* 's::md-trace))
                (u:text (format nil "hit ~s" (s::hitp *steps*)))
                (when (s::hitp *steps*)
                  (u:text (format nil " depth ~s~% normal ~a~% loc ~a"
                                  (s::depth *steps*)
                                  (s::normal *steps*)
                                  (s::location *steps*)))))
              (when *ref* (u::text *ref*))
              (when *gjk* (u::text *gjk*))
              (u:divider)
              (when *pushed* (u::text *pushed*))
              (when *pushed2* (u::text *pushed2*))
              (when *pushed3* (u::text *pushed3*)))
            #++
            (cond
              (*auto*
               (let ((u::*text-scale* 0.6))
                 (u:text (format nil "~a" *result*)))
               (let ((n 40)
                     (u::*text-scale* 0.5))
                 (loop repeat n
                       for r in *auto-log*
                       do (u:text r))
                 (when (> (length *auto-log*) (1+ n))
                   (setf (cdr (nthcdr (1- n) *auto-log*)) nil))))
              (t
               (let ((u::*text-scale* 0.6))
                 (u:text (format nil "~a" *result*))
                 (when *notes*
                   (u:text *notes*)))
               (let ((u::*text-scale* 0.6))
                 (u:text "ray=")
                 (u:text (format nil "~a" *ray-start*))
                 (u:text (format nil "~a" *ray-dir*))
                 (u:text "ref hit=")
                 (u:text (format nil "~a" *ref*))
                 (u:text (format nil "~a" *ref-normal*))))))
          )
        (u:window (#+ :wx 3/4 )
          (draw w now))))))

#++
(ql:quickload '(3b-glim 3b-glim-ui))
#++
(u:run 'foo :font "d:/tmp/arial32-5.json")
#++
(glut:show-window)
#++
(loop for i across glut::*id->window*
      when i do (glut:destroy-window (glut::id i)))
