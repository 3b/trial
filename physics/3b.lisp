#++
(ql:quickload '(3b-glim 3b-glim-ui trial-examples))
(defpackage #:gjk-test
  (:use #:cl)
  (:local-nicknames (#:a #:alexandria-2)
                    (#:u #:3b-glim-ui)
                    (#:glim #:3b-glim/s)
                    (#:g #:org.shirakumo.fraf.trial.gjk)
                    (#:trial #:org.shirakumo.fraf.trial)
                    (#:m #:org.shirakumo.fraf.math)))
(in-package #:gjk-test)

(defclass foo (u:window)
  ((flags :initform (make-hash-table) :reader flags)
   (c :initform 0 :accessor c)
   (a :initform 0 :accessor a)))

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
(defparameter *steps* 0)
(defparameter *test-idx* 0)
(defparameter *notes* nil)
(defvar *result* nil)
(defvar *obj* nil)
(defvar *done* nil)
(defvar *ref* (m:vec3))
(defvar *ref-normal* (m:vec3 0 0 1))
(defvar *tests* (list
                 (list (m:VEC3 3.059116 -3.2722962 2.4768581)
                       (m:VEC3 -0.62525314 0.42947507 -0.6516208))
                 (list (m:VEC3 3.4019854 2.2951577 -2.045888)
                       (m:VEC3 -0.7396158 -0.4090528 0.5344571))
                 (list (m:VEC3 -3.7934916 -3.4632647 -3.0540185)
                       (m:VEC3 0.4603436 0.56597906 0.6839236))
                 (list (m:VEC3 -2.1213768 3.444559 3.9394133)
                       (m:VEC3 0.66621196 -0.5006573 -0.5527241))
                 (list (m:VEC3 2.54517 2.7146091 3.980708)
                       (m:VEC3 -0.50233763 -0.72469383 -0.47167334))
                 (list (m:VEC3 -2.9625921 -2.9747531 -2.4236815)
                       (m:VEC3 0.4945116 0.6989014 0.5167157)
                       :sphere)
                 (list (m:VEC3 2.459015 2.4649944 -3.6213894)
                       (m:VEC3 -0.37049514 -0.63484466 0.6780159)
                       :cube)
                 (list (m:VEC3 -3.4184434 -2.5457313 2.1287484)
                       (m:VEC3 -0.69604313 0.5718277 -0.43420854)
                       :cylinder)
                 (list (m:VEC3 -2.442915 -3.481168 -3.4695456)
                       (m:VEC3 0.6848732 0.5035837 0.5266423)
                       :cylinder)
                 (list (m:VEC3 -3.1444643 -2.6627083 -3.7719972)
                       (m:VEC3 0.6371778 0.5365946 0.5532365)
                       :sphere)
                 (list (m:VEC3 3.8730307 -3.0099666 2.3163323)
                       (m:VEC3 -0.5739094 0.6138691 -0.54202646)
                       :cylinder)
                 (list (m:VEC3 -3.590032 -3.2336104 -3.945631)
                       (m:VEC3 0.49305567 0.43804753 0.75167173)
                       :cylinder)
                 (list (m:VEC3 3.1813586 2.4607022 3.5005364)
                       (m:VEC3 -0.5803402 -0.6600952 -0.4769482)
                       ;; ref is wrong here?
                       :cube)
                 (list (m:VEC3 2.971071 2.9997387 3.965216)
                       (m:VEC3 -0.528905 -0.6373113 -0.56044066)
                       ;;
                       :sphere)
                 (list (m:VEC3 2.5852838 -0.114477694 -0.7190808)
                       (m:VEC3 -0.49066052 -0.30163953 0.8174753)
                       :CYLINDER
                       (m:QUAT -0.22332716 0.10599959 0.76245177 -0.59796023))
                 (list (m:VEC3 47.579994 79.492386 42.75197)
                       (m:VEC3 -0.46105966 -0.7802588 -0.42263478)
                       :SPHERE
                       (m:QUAT -0.64127326 -0.3427993 0.62108123 0.29243007))
                 (list (m:VEC3 71.36629 9.424142 -101.26747)
                       (m:VEC3 -0.5771672 -0.0684088 0.81375563)
                       :CYLINDER
                       (m:QUAT 0.97279584 0.18697035 -0.07385229 0.11513563))
                 (list (m:VEC3 -11.526292 2.8096747 10.975833)
                       (m:VEC3 0.7242905 -0.26205805 -0.63775295)
                       :BOX (m:QUAT 0.0054854155 0.73955715 0.22828832 -0.6331742))
                 (list (m:VEC3 1.0096693 4.379136 3.0113254)
                       (m:VEC3 -0.17402828 -0.6926866 -0.6999281)
                       :BOX (m:QUAT 0.27514446 0.35296905 0.87930775 0.16286898))
                 (list (m:VEC3 -0.08129283 90.15353 55.43409)
                       (m:VEC3 -0.001282232 -0.844984 -0.53479004)
                       :CYLINDER (m:QUAT 0.35867596 -0.06621528 -0.69705415 0.6173188))
))
;;(sb-profile:profile "ORG.SHIRAKUMO.FRAF.TRIAL.GJK")
;;(sb-profile:report)
;;(sb-profile:reset)
;;(sb-profile:unprofile)
#++
(setf (values *ray-start* *ray-dir* *test-obj* *obj-orientation*)
      (values-list (nth 19 *tests*)))

#++
(setf (values *ray-start* *ray-dir* *test-obj* *obj-orientation*)
      (values (m:VEC3 -0.8475296 -0.514686 -2.360274)
              (m:VEC3 0.45012274 0.52356535 0.72337323)
              :CYLINDER  (m:QUAT 0.4119419 0.3932091 -0.8156164 0.10227609)))
#++
(setf (values *ray-start* *ray-dir* *test-obj* *obj-orientation*)
      (values (m:VEC3 -0.8475296 -0.514686 -2.360274)
              (m:VEC3 0.45012274 0.52356535 0.72337323)
              :sphere  (m:QUAT 0.4119419 0.3932091 -0.8156164 0.10227609)))
(defparameter *ray-start* (m:VEC3 3.059116 -3.2722962 2.4768581)
  ;(m:VEC3 3.4019854 2.2951577 -2.045888)
  ;(m:VEC3 -2.549998 0.0 1.6600008)
                                        ;(vec3 -3.5 -0.45 1.95)
                                        ;(vec3 -3.5 0 0)
                                        ;(vec3 0 0 -1)
  )
(defparameter *ray-dir* (m:VEC3 -0.62525314 0.42947507 -0.6516208)
  ;(m:VEC3 -0.7396158 -0.4090528 0.5344571)
  #++(m:VEC3 1 0 0))
(defvar *test-obj* :sphere)
(defvar *obj-orientation* (m:quat))
(m:vlength (m:vec3 -0.046570145 -0.6364093 0.7676495))
(defun run (steps)
  (let* ((g::*f* t)
         (g::*debug-state* *debug-state*)
         ;; so debug prints don't have package prefixes
         (*package* (find-package '#:g))
         (g::*steps* steps)
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
    (org.shirakumo.fraf.manifolds:transform-mesh
     (trial:vertices mesh) (m:qmat *obj-orientation*))
    (multiple-value-bind (r e) (catch :step
                                 (ignore-errors (trial:detect-hit ray obj)))
      (let ((g::*f* nil))
        (setf (values *ref* *ref-normal*)
              (ignore-errors
               (g::ref *ray-start* *ray-dir*
                           (aref (trial:physics-primitives obj) 0))))
)
      (format t "~s~%~s~%" r e )
      (setf *done* (or e (not (eql r :step) )))
      (setf *result* (or e r))
      (when (and r (not e) (not (eql r :step)))
        (setf *result*
              (format nil "~a~% d=~s~%"
                      (trial:hit-location r)
                      (when *ref*
                        (m:vdistance (trial:hit-location r) *ref*))
                      ))))))


(defparameter *zoom* 380)
(Defun draw-object ()
  (let ((s *zoom*)
        (g::*debug-state* *debug-state*))
    (labels ((d (k) (when g::*debug-state*
                      (gethash k g::*debug-state*)))
             (v (x y z)
               (u:vertex (* s x) (* s y) (* s z)))
             (vp (p)
               (if p
                   (v (m:vx p) (m:vy p) (m:vz p))
                   (v 0 0 0)))
             (vpa (p)
               (if p
                   (let ((p (g::point-a p)))
                     (v (m:vx p) (m:vy p) (m:vz p)))
                   (v 0 0 0)))
             #++(v? (p) (when p (v (m:vx p) (m:vy p) (m:vz p))))
             (v?d (k)
               (let ((p (d k)))
                 (when (and p (not (flag *w* k)))
                   (v (m:vx p) (m:vy p) (m:vz p))))))
      (when *obj*
        (let ((v (trial:vertices *obj*)))
          (glim:point-size 4)
          (glim:with-draw (:points :shader 'u::solid)
            (u:color 0.4 1 0.5 1)
            (loop for i below (length v) by 3
                  do (v (aref v (+ i 0))
                        (aref v (+ i 1))
                        (aref v (+ i 2)))))
          (when (d :x)
            (glim:point-size 6)
            (glim:with-draw (:points :shader 'u::solid)
              (u:color 1 0 1 1)
              (let ((x (d :x)))
                (loop for i below (length v) by 3
                      do (v (- (m:vx x) (aref v (+ i 0)))
                            (- (m:vy x) (aref v (+ i 1)))
                            (- (m:vz x) (aref v (+ i 2)))))))
            (glim:point-size 8)
            (glim:with-draw (:points :shader 'u::solid)
              (u:color 1 0 0 1)
              (v?d :x1)
              (u:color 0 1 0 1)
              (v?d :x2)
              (u:color 0 1 0 1)
              (v?d :v1)
              (u:color 1 1 1 1)
              (v?d :v2))
            )))
      (when *ref*
        (glim:with-draw (:lines :shader 'u:solid)
          (u:color 1 1 1 1)
          (let* ((d1 (m:vc (m:vc *ref-normal* (m:vec3 1 1 0)) *ref-normal*))
                 (d2 (m:vc (m:vc *ref-normal* (m:vec3 1 0 1)) *ref-normal*))
                 (dx (m:v* (m:nvunit (if (> (m:vsqrlength d1) (m:vsqrlength d2))
                                         d1 d2))
                           (/ 10 s)))
                 (dy (m:v* (m:nvunit (m:vc dx *ref-normal*))
                           (/ 10 s))))
            (vp (m:v- *ref* dx))
            (vp (m:v+ *ref* dx))
            (vp (m:v- *ref* dy))
            (vp (m:v+ *ref* dy)))))
      (when (d :v1)
        (destructuring-bind (&optional a b c d) (d :s)
          (u:color 0 1 1 1)
          (flet ((p (a s)
                   (when a
                     (glim:point-size s)
                     (glim:with-draw (:points :shader 'u:solid)
                       (vp a)))))
            (p a 30)
            (p b 20)
            ;;(u:color 1 0 1 1)
            (p c 15)
            (u:color 0 1 1 1)
            (p d 10))
          (glim:with-draw (:lines :shader 'u:solid)
            (unless (flag *w* :v2)
              (u:color 1 0 0 1)
              (u:vertex 0 0 0)
              (vp (d :v1)))
            (unless (flag *w* :v2)
              (u:color 1 0 0.7 1)
              (u:vertex 0 0 0)
              (vp (d :v2)))
            (unless (flag *w* :vw)
              (u:color 0 1 0 1)
              (u:vertex 0 0 0)
              (vp (d :vw)))
            (unless (flag *w* :vr)
              (u:color 0 0 1 1)
              (u:vertex 0 0 0)
              (vp (d :vr)))
            (when (and (d :rl) (d :rd))
              (u:color 0 1 0 1)
              (vp (d :rl))
              (u:color 0 1 1 1)
              (vp (m:v+ (d :rl) (d :rd)))

              (u:color 0 0 1 1)
              (vp (d :rl1))
              (u:color 0.5 0 1 1)
              (vp (m:v+ (d :rl1) (d :rd)))
              (u:color 0.3 0.1 0.4 0.3)
              (vp (m:v+ (d :rl) (d :rd)))
              (vp (m:v+ (d :rl) (m:v* (d :rd) 1000))))
            (when b
              (u:color 1 1 0 1)
              (vp a) (vp b)
              (when c
                (vp b) (vp c)
                (vp c) (vp a)
                (when d
                  (vp a) (vp d)
                  (vp b) (vp d)
                  (vp c) (vp d)
                  )))
            (when b
              (u:color 0.2 0.5 0.2 1)
              (vpa a) (vpa b)
              (when c
                (vpa b) (vpa c)
                (vpa c) (vpa a)
                (when d
                  (vpa a) (vpa d)
                  (vpa b) (vpa d)
                  (vpa c) (vpa d)
                  ))))))))

  )

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

#++
(list* "MAX" (reduce 'max *auto-distances*)
       (loop for i in '(a:median a:mean a:standard-deviation a:variance)
             collect (symbol-name i) collect (funcall i *auto-distances*)))
#++("MAX" 0.0011915698 "MEDIAN" 3.3717478e-7 "MEAN" 3.349292e-6
 "STANDARD-DEVIATION" 2.5538819e-5 "VARIANCE" 6.522312e-10)
#++("MAX" 6.0666306e-4 "MEDIAN" 3.3717478e-7 "MEAN" 3.411214e-6
 "STANDARD-DEVIATION" 2.2784705e-5 "VARIANCE" 5.191428e-10)
#++("MAX" 5.483376e-4 "MEDIAN" 3.5762787e-7 "MEAN" 3.435022e-6 "STANDARD-DEVIATION" 2.7118425e-5 "VARIANCE" 7.35409e-10)
#++
("MAX" 0.0055737784 "MEDIAN" 4.1295309e-7 "MEAN" 1.3124563e-4
 "STANDARD-DEVIATION" 5.6849327e-4 "VARIANCE" 3.231846e-7)
(defvar *auto-random* (make-random-state))
(defvar *random-queue* nil)
;(setf *random-queue* *old-errors*)

(defun random-test ()
  #++(let ((n 1))
       (setf (values *ray-start* *ray-dir*)
             (values-list (nth n *tests*))
             *test-obj* (or (third (nth n *tests*)) :sphere)))
  (setf *obj-orientation* (m:quat))
  (if *random-queue*
      (setf (values *ray-start* *ray-dir* *test-obj*)
            (values-list (pop *random-queue*)))
      (let ((*random-state* *auto-random* ))
        (flet ((rr () (* (+ 2 (random 2.0)) (signum (1- (random 2.0))))))

          (setf *ray-start* (m:vec3 (rr) (rr) (rr)))
          (setf *ray-dir* (m:nvunit (m:vec3 (rr) (rr) (rr))))
          (setf *test-obj* (ecase (random 3) (0 :sphere) (1 :box) (2 :cylinder)))

          )))
  )

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
             (let* ((g::*steps* steps)
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
               (let ((g::*f* nil))
                 (multiple-value-bind (r e)
                     (ignore-errors
                      (g::ref *ray-start* *ray-dir*
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
                       (if (> (gethash :steps g::*debug-state* 0) 2)
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
                             (fsteps (gethash :steps g::*debug-state* 0)))
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

(defun draw (window &optional (now 0))
  (when *auto* (auto-test now))
  ;; draw within a window in UI
  (u:canvas (:clear t :pixels t)
    (u::interact)
    (when (u::hot u::*context*)
      (incf (a window) (u::wheel u::*ui*)))
    (glim:with-pushed-matrix (:modelview)
      (glim:enable :depth-test)
      (u::with-clip-dimensions (wx wy)
        (let ((*random-state* (make-random-state *rs*))
              (w (* (min wx wy) 0.95))
              (z -0.01))
          (glim:translate (/ wx 2) (/ wy 2) 0)
          (glim:scale 1 1 (/ w))
          (glim:rotate (a window) 1 0 0)
          (glim:rotate (- (mod (* 5 now) 360)) 0 0 1)
          (glim:uniform 'u:modelview (glim:ensure-matrix :modelview))
          (glim:point-size 10)
          (draw-object)

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
                       (u:vertex w (* i s) z)))))))))

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
              (u:divider :pad 10)
              (u:text "..."))
            (u:row
              (u:text "zoom:")
              (flet ((f (n)
                       (when (u:button (format nil "~a~a"
                                               (if (eql n *zoom*)
                                                   "*" "")
                                               n))
                         (setf *zoom* n))))
                (f 200) (f 400) (f 1000) (f 3000) (f 10000) (f 50000)))
            (u:row (u:text "run random tests:")
              (when (u:button (if *auto* "stop" "start"))
                (setf *auto-state* :start)
                (setf *auto* (not *auto*)))
              (u:text (format nil "~s" *auto-state*)))
            (u:divider )
            (flet ((r () (setf *steps* 0 *done* nil)))
              (u:row
                (when (u:button "random")
                  (setf *notes* nil)
                  (random-test)
                  (r)
                  (run *steps*)
                  (let ((*package* (find-package '#:m)))
                    (format t "@ ~s~%  ~s~%" *ray-start* *ray-dir*)))
                (when (u:button (format nil "<"))
                  (setf *steps* (max 0 (1- *steps*)))
                  (run *steps*))
                (when (u:button (format nil "step (~a)"
                                        (if *done* "*" *steps*)))
                  (unless *done*
                    (incf *steps*)
                    (run *steps*)))
                (when (u:button (format nil "redo (~a)" *steps*))
                  (run *steps*))
                (when (u:button "restart")
                  (r)
                  (run *steps*))
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
            (u:divider)
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

