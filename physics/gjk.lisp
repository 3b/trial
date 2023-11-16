;;; This implementation was initially roughly based on the following implementation
;;;   https://github.com/Another-Ghost/3D-Collision-Detection-and-Resolution-Using-GJK-and-EPA
;;; Though it has been largely modified and extended.
;;; The raycast implementation is largely based on "Ray Casting against General Convex Objects
;;; with Application to Continuous Collision Detection" by Gino Van Den Bergen available at
;;;   http://dtecta.com/papers/jgt04raycast.pdf

(defpackage #:org.shirakumo.fraf.trial.gjk
  (:use #:cl #:org.shirakumo.fraf.math)
  (:export
   #:detect-hits
   #:support-function))

(in-package #:org.shirakumo.fraf.trial.gjk)

(defconstant GJK-ITERATIONS 64)
(defconstant EPA-ITERATIONS 64)
(defconstant EPA-TOLERANCE 0.0001)
(defconstant EPA-MAX-FACES 64)
(defconstant EPA-MAX-LOOSE-EDGES 32)

;;;; GJK main algorithm
;; TODO: avoid consing from v-

(declaim (ftype (function (vec3 vec3 vec3 vec3 &optional vec3) vec3) barycentric))
(defun barycentric (a b c p &optional (res (vec3)))
  (declare (optimize speed (safety 0)))
  (declare (type vec3 a b c p res))
  ;; Compute the barycentric coordinates of P within the triangle spanned by A B C
  (let* ((v0 (v- b a))
         (v1 (v- c a))
         (v2 (v- p a))
         (d00 (v. v0 v0))
         (d01 (v. v0 v1))
         (d11 (v. v1 v1))
         (d20 (v. v2 v0))
         (d21 (v. v2 v1))
         (denom (- (* d00 d11) (* d01 d01))))
    (declare (dynamic-extent v0 v1 v2))
    (if (<= denom 0.0000001f0)
        (vsetf res 1 0 0)
        (let ((v (/ (- (* d11 d20) (* d01 d21)) denom))
              (w (/ (- (* d00 d21) (* d01 d20)) denom)))
          (vsetf res (- 1f0 v w) v w)))))

(barycentric (VEC3 0.16323563 0.0949059 -0.07241821)
             (VEC3 0.07622701 -0.071716934 -0.03046)
             (VEC3 -0.03091523 0.114028014 0.0026008487)
             (vec3 0 0 0))
(VEC3 -0.19839776 0.6352216 0.56317616)

(declaim (inline plane-normal))
(defun plane-normal (a b c &optional (res (vec3)))
  (declare (optimize speed (safety 0)))
  (declare (type vec3 a b c res))
  (let ((ba (v- b a))
        (ca (v- c a)))
    (declare (dynamic-extent ba ca))
    (nvunit* (!vc res ba ca))))

(defun plane-point (a b c &optional (res (vec3)))
  (declare (optimize speed (safety 0)))
  (declare (type vec3 a b c res))
  ;; Compute "the" central point of the plane spanned by A B C via its normal
  ;; This is the same as computing the plane, then projecting the zero point
  ;; onto that plane.
  (let* ((normal (plane-normal a b c res))
         (offset (v. normal a)))
    (nv* normal offset)))

(declaim (inline point))
(defstruct (point
            (:constructor point (&optional (varr3 (make-array 3 :element-type 'single-float))))
            (:include vec3)
            (:copier NIL)
            (:predicate NIL))
  (a (vec3 0 0 0) :type vec3)
  (b (vec3 0 0 0) :type vec3))

(declaim (inline p<-))
(defun p<- (target src)
  (declare (type point target src))
  (v<- target src)
  (v<- (point-a target) (point-a src))
  (v<- (point-b target) (point-b src)))

(defun search-point (p +dir a b)
  (declare (optimize speed))
  (declare (type point p))
  (declare (type vec3 +dir))
  (let ((-dir (v- +dir)))
    (declare (dynamic-extent -dir))
    (%support-function a -dir (point-a p))
    (%support-function b +dir (point-b p))
    (!v- p (point-b p) (point-a p))))

(trial:define-hit-detector (trial:primitive trial:primitive)
  (setf trial:start (detect-hits a b trial:hits trial:start trial:end)))

(defun detect-hits (a b hits start end)
  (declare (type trial:primitive a b))
  (declare (type (unsigned-byte 32) start end))
  (declare (type simple-vector hits))
  (declare (optimize speed))
  (when (<= end start)
    (return-from detect-hits start))
  (let ((hit (aref hits start)))
    (let ((s0 (point)) (s1 (point)) (s2 (point)) (s3 (point)) (dir (point)) (s12 (point)))
      (declare (dynamic-extent s0 s1 s2 s3 dir s12))
      (trial::global-location a dir)
      (trial::global-location b s0)
      (nv- dir s0)
      (search-point s2 dir a b)
      (!v- dir s2)
      (search-point s1 dir a b)
      (!v- s12 s2 s1)
      (cond ((< (v. s1 dir) 0)
             start)
            (T
             (!vc dir (!vc dir s12 (v- s1)) s12)
             (when (v= 0 dir)
               (!vc dir s12 +vx3+)
               (when (v= 0 dir)
                 (!vc dir s12 +vz3+)))
             (loop with dim of-type (unsigned-byte 8) = 2
                   for i from 0 below GJK-ITERATIONS
                   do (search-point s0 dir a b)
                      (when (< (v. s0 dir) 0)
                        (return start))
                      (incf dim)
                      (cond ((= 3 dim)
                             (setf dim (update-simplex s0 s1 s2 s3 dir)))
                            ((null (test-simplex s0 s1 s2 s3 dir))
                             (setf dim 3))
                            (T
                             (epa s0 s1 s2 s3 a b hit)
                             (trial:finish-hit hit a b)
                             (return (1+ start))))))))))

(defvar *f* nil)
(defvar *steps* nil)
(defun v (f &rest r)
  (when *f*
    (apply 'format t f r)))
(defparameter *debug-state* (make-hash-table))
(defun d (k v)
  (when *debug-state*
    (if (eql k nil)
        (clrhash *debug-state*)
        (setf (gethash k *debug-state*)
              (if (vec-p v) (vcopy v) v)))))
#++
(trial:define-ray-test trial:primitive ()
  (let* ((tt 0.0) ;;λ?
         ;; s = ray-location
         (x (vcopy ray-location))
         (dx (vec3))
         ;; n = ray-normal
         (w (vec3)) ;; w
         (p (vec3)) ;; p
         (dir x)    ;; v?
         (dim 0)    ;; /P/ = dim + s0,s1,s2,s3
         (maxdist 1.0)
         (s0 (vec3))
         (s1 (vec3))
         (s2 (vec3))
         (s3 (vec3)))
    (declare (dynamic-extent x dir w p s0 s1 s2 s3))
    (declare (type (unsigned-byte 8) dim))
    (vsetf ray-normal 0 0 0)
    (v "rloc =~s dir=~s~% p=~s~% dir=~s~%"
       ray-location ray-direction p dir)
    (loop for i from 0 below 5 ;GJK-ITERATIONS
          while (progn
                  (v "~%iteration ~s/~s:~%" i gjk-iterations)
                  (v "?(<= ~s ~s) ~s~%"
                     (* 0.000001f0 maxdist) (vsqrlength dir)
                     (<= (* 0.000001f0 maxdist) (vsqrlength dir))
                     )
                  (<= (* 0.000001f0 maxdist) (vsqrlength dir)))
          do
             (support-function trial:primitive dir p)
             (!v- w x p)
             (v "w=~s~%x=~s~%p=~s~%" w x p)
             (let ((vw (v. dir w))
                   (vr (v. dir ray-direction)))
               (v ":vw=~s, vr=~s~%" vw vr)
               (cond ((<= vw 0))
                     ((<= 0 vr)
                      (v "< 0 vr ~s~%" vr)
                      (return NIL))
                     (T
                      (let ((dt (/ vw vr)))
                        (decf tt dt)
                        (!v* dx ray-direction (- dt)))
                      (!v+* x ray-location ray-direction tt)
                      (v<- ray-normal dir)
                      (v "tt=>~s~% x=~s~% ray-normal=~s~% dx=~s~%"
                         tt x ray-normal dx)))
               ;; Add p to the simplex
               (incf dim)
               (v<- s3 s2)
               (v<- s2 s1)
               (v<- s1 s0)
               (v<- s0 p)
               (v "dim=~s~% s0=~s~% s1=~s~% s2=~s~% s3=~s~% p=~s~%"
                  dim s0 s1 s2 s3 p)
               ;; Update dir as the closest point to zero within the simplex
               (v<- dir s0)
               (v "  dir=~s~%" dir)
               (when (and (< 1 dim) (< (vsqrlength s1) (vsqrlength dir)))
                 (v "s1")
                 (v<- dir s1))
               (when (and (< 2 dim) (< (vsqrlength s2) (vsqrlength dir)))
                 (v "s2")
                 (v<- dir s2))
               (when (and (< 3 dim) (< (vsqrlength s3) (vsqrlength dir)))
                 (v "s3")
                 (v<- dir s3))
               (v "dir=~s~%" dir)
               ;; Reduce the simplex
               (when (= 4 dim)
                 (decf dim)
                 (when (v= s3 dir)
                   (v "<dim=4,s3=dir>")
                   (v<- s0 s3))
                 (v "dim->~s,s0->~s~%" dim s0)))
          finally (progn
                    (v "---~%final: ~s ~s~%" ray-normal tt)
                    (nvunit* ray-normal)
                    (return tt)))))

;;; from
;; Mattia Montanari, Nik Petrinic, and Ettore Barbieri. 2017.
;; Improving the GJK algorithm for faster and more reliable distance
;; queries between convex objects.
;; ACM Trans. Graph. 36, 3, Article 30 (June 2017)
;; DOI: http://dx.doi.org/10.1145/3083724
(defun signed-volumes (dim s0 s1 s2 s3 dir)
  ;; returns # of dimensions remaining, and any remaining points in
  ;; s1,s2,s3 (unless 3-simplex contains origin in which case 4 is
  ;; returned and s0-3 are unchanged). also returns point on simplex
  ;; nearest to origin in DIR
  (float-features:with-float-traps-masked (:divide-by-zero
                                           :invalid)
    (v "@@@ sv ~s~%" dim)
    (ecase dim
     (1
      (p<- s1 s0)
      ;(!v- dir s1)
      (v<- dir s1)
      1)
     (2 (sv1d s0 s1 s2 s3 dir))
     (3 (sv2d s0 s1 s2 s3 dir))
     (4 (sv3d s0 s1 s2 s3 dir)))))

(defun sv-compare-signs (a b)
  #++ (or (and (< a 0) (< b 0))
          (and (< 0 a) (< 0 b)))
  (let ((ϵ 0.00005))
    (or (and (< a (- ϵ)) (< b (- ϵ)))
        (and (< ϵ a) (< ϵ b)))))

(defun sv3d (s0 s1 s2 s3 dir)
  (let* ((m (mat4 (vx s0) (vx s1) (vx s2) (vx s3)
                  (vy s0) (vy s1) (vy s2) (vy s3)
                  (vz s0) (vz s1) (vz s2) (vz s3)
                  1 1 1 1))
         (c (vec4))
         (mdet 0f0)
         (flat nil)
         (ϵ 0.00001))
    (declare (dynamic-extent m c))
    (loop for j below 4
          for cj = (mcofactor m 3 j)
          do (setf (aref (varr c) j) cj)
             (incf mdet cj))
    (setf flat (< (abs mdet) ϵ))
    (v "3d:~%")
    (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
    (v "  c=~s~%  det=~s (flat ~s)~% " c mdet flat)

    (cond
      ;; contained in simplex
      ((and (not flat)
            (loop for cj across (varr c) always (sv-compare-signs mdet cj)))
       (!v* dir s0 (/ (vx c) mdet))
       (!v+* dir dir s1 (/ (vy c) mdet))
       (!v+* dir dir s2 (/ (vz c) mdet))
       (!v+* dir dir s3 (/ (vw c) mdet))
       4)
      ;; otherwise find best of the faces
      (t
       (let (;;
             (d most-positive-single-float)
             ;; best result seen so far
             (best-dim 0)
             (best-dir (vec3))
             (b0 (point))
             (b1 (point))
             (b2 (point))
             ;; s* passed to particular call of sv2d
             (cdir (vec3))
             (c0 (point))
             (c1 (point))
             (c2 (point))
             (c3 (point)))
         (declare (dynamic-extent best-dir cdir b0 b1 b2 c0 c1 c2 c3))
         (v " faces det~s, c~s (flat ~s)~%" mdet c flat)
         (loop for j from 1 to 3
               when (or flat
                        (not (sv-compare-signs mdet (aref (varr c) j))))
                 do (p<- c0 (if (= j 0) s1 s0))
                    (p<- c1 (if (< j 2) s2 s1))
                    (p<- c2 (if (< j 3) s3 s2))
                    (let ((r (sv2d c0 c1 c2 c3 cdir))
                          (d* (vsqrlength cdir)))
                      (v "~a d ~s, #~s, dir ~s~%"
                         (if (< d* d) "!!3" "??3")
                         d* r cdir)
                      (when (< d* d)
                        (setf d d*
                              best-dim r)
                        (v "3c1=~s~%" c1)
                        (v<- best-dir cdir)
                        ;; results are in c1[,c2[,c3]]
                        (p<- b0 c1)
                        (when (> r 1) (v "3c2=~s~%" c2)(p<- b1 c2))
                        (when (> r 2) (v "3c3=~s~%" c3) (p<- b2 c3)))))
         ;; should always find a solution
         (assert (/= d most-positive-single-float))
         ;; copy results to output
         (v<- dir best-dir)
         (p<- s1 b0)
         (when (> best-dim 1) (p<- s2 b1))
         (when (> best-dim 2) (p<- s3 b2))
         best-dim)))))

#++
(defun sv2d (s1 s2 s3 s4 dir)
  ;; note 1-based S* because i keep making mistakes transcribing
  ;; algo. todo: search and replace once it works
                                        ;(declare (type point s1 s2 s3 s4))
  (let* ((n (vc (v- s2 s1) (v- s3 s1)))
         (p₀ (v* n (/ (v. s1 n)
                      (vsqrlength n))))
         (μₘₐₓ 0)
         (j -1)
         (flat nil)
         (ϵ 0.00001))
    (declare (dynamic-extent n p₀))
    (v "2d:~%")
    (v "  s0=~s~%  s1=~s~%  s2=~s~%" s1 s2 s3)
    (v "  n=~s~%  p₀=~s~%" n p₀)
    (loop ;;with k = 1 ;; i,k,l 0-based
          ;;with l = 2
          for i below 3
          for k = (mod (1+ i) 3)
          for l = (mod (1+ k) 3)
          for μ = (- (+ (* (vref s2 k) (vref s3 l))
                        (* (vref s1 k) (vref s2 l))
                        (* (vref s3 k) (vref s1 l)))
                     (* (vref s2 k) (vref s1 l))
                     (* (vref s3 k) (vref s2 l))
                     (* (vref s1 k) (vref s3 l)))
          do (v " ikl=~s,~s,~s,μ=~s~a~%" i k l μ (if (> (abs μ) (abs μₘₐₓ)) "!!" ""))
          when (> (abs μ) (abs μₘₐₓ))
            do (setf μₘₐₓ μ
                     j i)
               ;;do (shiftf k l i)
          )
    (let ((ab (v- s2 s1))
          (ac (v- s3 s1)))
      (v " ab=~s,ac=~s~%" ab ac)
      (v "  x=~s~%" (vx (vc (v_yz ab) (v_yz ac))))
      (v "  y=~s~%" (vy (vc (vx_z ab) (vx_z ac))))
      (v "  z=~s~%" (vz (vc (vxy_ ab) (vxy_ ac)))))
    (let ((ab (v- s1 s3))
          (ac (v- s2 s3)))
      (v " ab=~s,ac=~s~%" ab ac)
      (v "  x=~s~%" (vx (vc (v_yz ab) (v_yz ac))))
      (v "  y=~s~%" (vy (vc (vx_z ab) (vx_z ac))))
      (v "  z=~s~%" (vz (vc (vxy_ ab) (vxy_ ac)))))
    (let ((ab (v- s2 s3))
          (ac (v- s1 s3)))
      (v " ab=~s,ac=~s~%" ab ac)
      (v "  x=~s~%" (vx (vc (v_yz ab) (v_yz ac))))
      (v "  y=~s~%" (vy (vc (vx_z ab) (vx_z ac))))
      (v "  z=~s~%" (vz (vc (vxy_ ab) (vxy_ ac)))))
    (setf flat (< (abs μₘₐₓ) ϵ))
    (v " j=~s, flat=~s~%" j flat)
    (flet ((v2x (v i)
             (ecase i
               (0 (vyz v))
               (1 (vxz v))
               (2 (vxy v))
               ))
           ;; factor out edge test so we can call it directly for
           ;; degenerate case where v2x won't work
           (edges (c)
             (let ((d most-positive-single-float)
                   ;; best result seen so far
                   (best-dim 0)
                   (best-dir (vec3))
                   (b0 (point))
                   (b1 (point))
                   ;; s* passed to particular call of sv1d
                   (cdir (vec3))
                   (c0 (point))
                   (c1 (point))
                   (c2 (point))
                   (c3 (point)))
               (declare (dynamic-extent best-dir cdir b0 b1 c0 c1 c2 c3))
               (v " edges μₘₐₓ~s,c~s (flat ~s)~%" μₘₐₓ c flat)
               (loop for j from 1 to 2 ;; 0-based
                     when (or (not c)
                              flat
                              (not (sv-compare-signs μₘₐₓ (aref (varr c) j))))
                       ;; 1-based s here
                       do (p<- c0 s1)
                          (p<- c1 (if (= j 1) s3 s2))
                          (let ((r (sv1d c0 c1 c2 c3 cdir))
                                (d* (vsqrlength cdir)))
                            (format t "edge ~s = ~s (~s)~a~%" j r d*
                                    (if (< d* d) "!!" ""))
                            (when (< d* d)
                              (setf d d*
                                    best-dim r)
                              (v<- best-dir cdir)
                              ;; results are in c1[,c2]
                              (p<- b0 c1)
                              (when (> r 1) (p<- b1 c2))
                              (v "!! d~s, #~s dir=~s~%  b0=~s~%  b1=~s~%"
                                 d best-dim best-dir b0 b1))))
               ;; should always find a solution
               (assert (/= d most-positive-single-float))
               ;; copy results to output
               (v<- dir best-dir)
               ;; 1-based s here
               (p<- s2 b0)
               (when (> best-dim 1) (p<- s3 b1))
               (v "<- dir~s #~s~% @ b1~s~% b2~s~%" dir best-dim s2 s3)
               best-dim)))
      (declare (inline v2x))
      (if (minusp j)
          (edges nil)
          (let* ((s1₂ (v2x s1 j))
                 (s2₂ (v2x s2 j))
                 (s3₂ (v2x s3 j))
                 (p₀₂ (v2x p₀ j))
                 (s* (vector s1₂ s2₂ s3₂))
                 (c (vec4)))
            (declare (dynamic-extent s1₂ s2₂ s3₂ p0₂ s*))
            (loop with k = 1 ;; i,k,l 0-based
                  with l = 2
                  ;; paper uses j here and in setf, but i in shiftf below?
                  for i from 1 to 2
                                        ;for k = (mod (1+ i) 3)
                                        ;for l = (mod (1+ k) 3)
                  do (setf (vref c i)
                           (* (expt -1 (1+ i))
                              (- (+ (* (vx p₀₂) (vy (aref s* k)))
                                    (* (vy p₀₂) (vx (aref s* l)))
                                    (* (vx (aref s* k)) (vy (aref s* l))))
                                 (* (vx p₀₂) (vy (aref s* l)))
                                 (* (vy p₀₂) (vx (aref s* k)))
                                 (* (vx (aref s* l)) (vy (aref s* k))))))
                     (format t "  ikl=~s,~s,~s, c=~s~%" i k l (vref c i))
                     (shiftf k l k)
                  )
;;; acy*apz-acz*apy
;;; -s1z*(s3y-s1y) + s1y*(s3z-s1z)
            ;; s1z*s1y + s1y*s3z - s1y*s1z - s1z*s3y

            (flet ((v2x (v)
                     (ecase j
                       (0 (v_yz v))
                       (1 (vx_z v))
                       (2 (vxy_ v))
                       )))
              (v "  μₘₐₓ=~s, j=~s, c=~s~%" μₘₐₓ j c)
              (let* ((ca (v- s1 s3))
                     (cb (v- s2 s3))
                     (cp (v- s3)) ;; = (- p₀ s3)
                     (μₘₐₓ2 (vref (vc (v2x ca) (v2x cb)) j))
                     (n (v* (vunit (vc ca cb))
                            ;(expt -1 j)
                            )
                        #++(v* (ecase j
                                 (0 +vx3+)
                                 (1 +vy3+)
                                 (2 +vz3+))
                               1 #++(if (minusp (vref s1 j)) -1.0 1.0)))
                     (c1 (vref (vc (v2x ca) (v2x cp)) j))
                     (c2 (vref (vc (v2x cp) (v2x cb)) j)))
                (v " μₘₐₓ2=~s n=~s~%" μₘₐₓ2 n)
                (v "  ab=~s~%  ac=~s~%  ap=~s~%" ca cb cp)
                (v " c₁=~s (~s)~%" c1 (vc (v2x ca) (v2x cp)))
                (v " c₂=~s (~s)~%" c2 (vc (v2x cp) (v2x cb)))
                (progn
                  (setf μₘₐₓ μₘₐₓ2)
                  (setf (vref c 1) c1)
                  (setf (vref c 2) c2))))
            (v "  p₀₂=~s~%" p₀₂)
            (v "  s0=~s~%  s1=~s~%  s2=~s~%" s1₂ s2₂ s3₂)
            (cond
              ;; support is entire triangle, keep 3 points
              ((and (sv-compare-signs μₘₐₓ (vref c 1))
                    (sv-compare-signs μₘₐₓ (vref c 2))
                    (not (zerop μₘₐₓ)))
               ;; algorithm is unclear here... compares sign for 'all j'
               ;; is that 1-3 or just the 2 we calculated above? and C₁ is
               ;; used for λ₁ but wasn't calculated?
               (let* ((λ₂ (/ (vref c 1) μₘₐₓ))
                      (λ₃ (/ (vref c 2) μₘₐₓ))
                      (λ₁ (- 1 λ₂ λ₃)))
                 (v "  λ = ~s,~s,~s~%" λ₁ λ₂ λ₃)
                 (assert (<= 0 λ₁ 1))
                 ;; 1-based s here
                 (!v* dir s3 λ₁)
                 (!v+* dir dir s2 λ₂)
                 (!v+* dir dir s1 λ₃)
                 (p<- s4 s3)
                 (p<- s3 s2)
                 (p<- s2 s1)
                 (v "  3dir<-~s~%" dir)
                 (v "   s1=~s~%   s2=~s~%   s3=~s~%" s2 s3 s4)
                 3))
              ;; try edges
              (t
               (edges c))))))))


(defun sv2d (s0 s1 s2 s3 dir)
  (let* ((ab3 (v- s1 s0))
         (ac3 (v- s2 s0))
         (bc3 (v- s2 s1))
         (n (vc ac3 ab3))
         (l² (vsqrlength n))
         (p₀ (vec3))
         (μₘₐₓ 0)
         (j -1)
         (flat nil)
         (ϵ 0.00001))
    (declare (dynamic-extent n p₀))
    (v "2d:~%")
    (v "  s0=~s~%  s1=~s~%  s2=~s~%" s0 s1 s2)
    ;; project origin onto plane (if possible)
    (if (< l² (expt ϵ 2))
        (setf flat t)
        (setf p₀ (v* n (/ (v. s0 n) (vsqrlength n)))))
    (v "  n=~s (~s (~s))~%  p₀=~s~%" n l² flat p₀)

    ;; if too flat, just pick best result from 1d test against all edges
    (when flat
      (let ((d most-positive-single-float)
            ;; best result seen so far
            (best-dim 0)
            (best-dir (vec3))
            (b0 (point))
            (b1 (point))
            ;; s* passed to particular call of sv1d
            (cdir (vec3))
            (c0 (point))
            (c1 (point))
            (c2 (point))
            (c3 (point)))
        (declare (dynamic-extent best-dir cdir b0 b1 c0 c1 c2 c3))
        (v " edges~%")
        (loop for j from 0 to 2
              do (p<- c0 (if (= j 0) s1 s0))
                 (p<- c1 (if (< j 2) s2 s1))
                 (let ((r (sv1d c0 c1 c2 c3 cdir))
                       (d* (vsqrlength cdir)))
                   (v "edge ~s = ~s (~s)~a~%" j r d*
                      (if (< d* d) "!!" ""))
                   (when (< d* d)
                     (setf d d*
                           best-dim r)
                     (v<- best-dir cdir)
                     ;; results are in c1[,c2]
                     (v "2c1=~s~%" c1)
                     (p<- b0 c1)
                     (when (> r 1) (v "2c2=~s~%" c2) (p<- b1 c2))
                     (v "!! d~s, #~s dir=~s~%  b0=~s~%  b1=~s~%"
                        d best-dim best-dir b0 b1))))
        ;; should always find a solution
        (assert (/= d most-positive-single-float))
        ;; copy results to output
        (v<- dir best-dir)
        (p<- s1 b0)
        (when (> best-dim 1) (p<- s2 b1))
        (v "<- dir~s #~s~% @ b1~s~%   b2~s~%" dir best-dim s1 s2)
        (return-from sv2d best-dim)))


    ;; otherwise pick axis-aligned plane with largest projection
    (loop for i below 3
          for μ = (flet ((v2x (v)
                           (ecase i
                             (0 (v_yz v))
                             (1 (vx_z v))
                             (2 (vxy_ v)))))
                    (vref (vc (v2x ab3) (v2x ac3)) i))
          do (v " i=~s,μ=~s~a~%" i μ (if (> (abs μ) (abs μₘₐₓ)) "!!" ""))
          when (> (abs μ) (abs μₘₐₓ))
            do (setf μₘₐₓ μ
                     j i))
    (setf flat (< (abs μₘₐₓ) ϵ))
    (v " j=~s, flat=~s~%" j flat)
    (assert (not flat))
    ;; calculate barycentric coordinates on that plane
    (flet ((v2x (v i)
             (ecase i
               (0 (vyz v))
               (1 (vxz v))
               (2 (vxy v))))
           (v2c (a b)
             (vz (vc (vec3 a 0) (vec3 b 0)))))
      (declare (inline v2x))
      (let* ((ab (v2x ab3 j))
             (ac (v2x ac3 j))
             ;(bc (v2x bc3 j))
             (p (v2x p₀ j))
             ;(p₀ (vec2 0 0))
             (ap (v- p (v2x s0 j)))
             (pb (v- p (v2x s1 j)))
             (pc (v- p (v2x s2 j)))
             (abc (v2c ab ac))
             ;(c (vec4))
             (λ₁ (/ (v2c pb pc) abc))
             (n₁ (<= λ₁ 0))
             (λ₂ (/ (v2c ap ac) abc))
             (n₂ (<= λ₂ 0))
             (λ₃ (/ (v2c ab ap) abc))
             (n₃ (<= λ₃ 0))
             (out (+ (if n₁ 1 0) (if n₂ 1 0) (if n₃ 1 0))))
        (v "  λ = ~s,~s,~s `= ~s~%" λ₁ λ₂ λ₃ (+ λ₁ λ₂ λ₃))
        #++(assert (< (- 1 (* 100 ϵ)) (+ λ₁ λ₂ λ₃) (+ 1 (* 100 ϵ))))
        ;; seen ~0.003 error here
        (assert (< (- 1 0.01) (+ λ₁ λ₂ λ₃) (+ 1 0.01)))
        (cond
          ;; support is entire triangle
          ((and (zerop out)
                (< λ₁ 1)
                (< λ₂ 1)
                (< λ₃ 1))
           (!v* dir s0 λ₁)
           (!v+* dir dir s1 λ₂)
           (!v+* dir dir s2 λ₃)
           (p<- s3 s2)
           (p<- s2 s1)
           (p<- s1 s0)
           (v "  3dir<-~s~%" dir)
           (v "   s1=~s~%   s2=~s~%   s3=~s~%" s1 s2 s3)
           3)
          ((= out 1)
           ;; support is line opposite negative λ
           (flet ((edge (a b m)
                    (let* ((t₀ (/ (v. m (v- a))
                                  (v. m m))))
                      (v " a=~s~% b=~s~% m=~s~% t₀=~s~%" a b m t₀)
                      (!v+* dir a m t₀))
                    (p<- s2 b)
                    (p<- s1 a)
                    (v "  3dir2<-~s~%" dir)
                    (v "   s1=~s~%   s2=~s~%" s1 s2)
                    2))
             (cond
               (n₁ (edge s1 s2 bc3))
               (n₂ (edge s0 s2 ac3))
               (n₃ (edge s0 s1 ab3)))))
          ((= out 2)
           ;; support is point with positive λ
           (cond
             ((not n₁)
              (v<- dir s0)
              (p<- s1 s0)
              1)
             ((not n₂)
              (v<- dir s1)
              (p<- s1 s1)
              1)
             ((not n₃)
              (v<- dir s2)
              (p<- s1 s2)
              1)))
          (t (error "shouldn't get here?")))

        
        ))))
#++
(defun sv1d (s0 s1 s2 s3 dir)
  (declare (ignore s3))
    (v "1d:~%")
    (v "  s0=~s~%  s1=~s~%" s0 s1)
  (let* ((tt (v- s1 s0))
         #++(p₀ (v+ s1 (v* tt (/ (v. s1 tt)
                                 (v. tt tt)))))
         #++ (p₀  (v* tt (/ (v. (v- s0) tt)
                            (v. tt tt))))
         (p₀ (let ((p (vec3))
                   (b s0)
                   (m tt))
               (v " b=~s,m=~s~%" b m)
               (v " (p-b)=~s, m.(p-b)=~s, m.m=~s~%"
                  (v- p b) (v. m (v- p b)) (v. m m))
               (let ((t0 (/ (v. m (v- p b))
                            (v. m m))))
                 (v " t0=~s, m*t0=~s,~% p0=~s~%" t0 (v* m t0)
                    (v+ b (v* m t0)))
                (v+ b (v* m t0)))))
         (μₘₐₓ 0f0)
         (ii -1))
    (loop for i below 3
          for μ = (- (vref s0 i) (vref s1 i))
          do (v " i=~s,μ=~s~a~%" i μ (if (> (abs μ) (abs μₘₐₓ)) "!!" ""))
          when (> (abs μ) (abs μₘₐₓ))
            do (setf μₘₐₓ μ
                     ii i))
    (flet ((sk (k)
             (ecase k (0 s0) (1 s1))))
      (declare (inline sk))
      (let ((c (vec3)))
        (declare (dynamic-extent c))
        (unless (= ii -1)
          (loop for ii below 3
                do (loop with k = 1 ;; j,k 0-based
                         for j from 0 to 1
                         do (setf (vref c j)
                                  (*    ;(expt -1 (1+ j))
                                   (- (vref (sk k) ii)
                                      (vref p₀ ii))))
                            (setf k j))
                   (v " c~s = ~s~%" ii c))
          (loop with k = 1 ;; j,k 0-based
                for j from 0 to 1
                do (setf (vref c j)
                         (* (expt -1 (1+ j))
                          (- (vref (sk k) ii)
                             (vref p₀ ii))))
                   (setf k j)))
        (v "  μₘₐₓ=~s, ii=~s, c=~s~%" μₘₐₓ ii c)
        (v "  p₀=~s~%" p₀)
        (cond
          ;; support is line
          ((and (not (zerop μₘₐₓ))
                (/= ii -1)
                (sv-compare-signs μₘₐₓ (vref c 0))
                (sv-compare-signs μₘₐₓ (vref c 1)))
           (let* ((λ₁ (/ (vref c 0) μₘₐₓ))
                  (λ₂ (/ (vref c 1) μₘₐₓ)))
             (p<- s2 s1)
             (p<- s1 s0)
             (v "  λ=~s ~s~%" λ₁ λ₂)
             (!v* dir s1 λ₁)
             (!v+* dir dir s2 λ₂)
             (v "  2dir<- ~s~%" dir)
                                        ;(!v- dir dir)
             2))
          ;; support is point
          (t
           (p<- s1 s0)
           (!v- dir s1)
           (v "  1dir<- ~s~%" dir)
           1))))))

(defun sv1d (s0 s1 s2 s3 dir)
  (declare (ignore s3))
  (v "1d:~%")
  (v "  s0=~s~%  s1=~s~%" s0 s1)
  (let* ((m (v- s1 s0))
         (mm (v. m m))
         (ϵ 0.00001))
    (v "  mm=~s,m=~s~%" mm m)
    (cond
      ((< (abs mm) ϵ)
       ;; degenerate segment, just return a point
       (v " dir=s0=~s~%" s0)
       (v<- dir s0)
       (p<- s1 s0)
       1)
      (t
       (let* ((t₀ (/ (v. m (v- s0))
                     (v. m m))))
         (v " a=~s~% b=~s~% t₀=~s~%" s0 s1 t₀)
         (cond
           ((<= t₀ 0)
            ;; start point
            (v " dir1a=s0=~s~%" s0)
            (v<- dir s0)
            (p<- s1 s0)
            1)
           ((<= 1 t₀)
            ;; end point
            (v " dir1b=s0=~s~%" s1)
            (v<- dir s1)
            ;(p<- s1 s1)
            1)
           (t
            ;; keep both points
            (!v+* dir s0 m t₀)
            (p<- s2 s1)
            (p<- s1 s0)
            (v " 1dir2<-~s~%" dir)
            (v "   s1=~s~%   s2=~s~%" s1 s2)
            2)))))))

#++
(trial:define-ray-test trial:primitive ()
  (let* (;; inputs
         (s ray-location)
         (r ray-direction)
         ;; output n = ray-normal
         ;; state
         (λ 0.0)
         (x (vcopy s))
         (w (vec3))
         (p (vec3))
         (v (vcopy x))
         ;; amount we moved X, used to update cso
         (dx (vec3))
         ;; set P
         (dim 0) ;; # of valid elements in P
         (s0 (point))
         (s1 (point))
         (s2 (point))
         (s3 (point))
         ;; misc
         (stuck nil)
         (maxdist 1.0))
                                        ;(declare (dynamic-extent λ x v w p s0 s1 s2 s3))
    (declare (type (unsigned-byte 8) dim))
    (vsetf ray-normal 0 0 0)
    (v "~&rloc =~s dir=~s~% p=~s~% v=~s~% x=~s~%"
       ray-location ray-direction p v x)
;;;; calculate first 2 points directly
    ;;(setf dim 1)
;;;; first point on simplex = support in arbitrary direction:
    ;;(support-function trial:primitive v s1)
    ;;(!v- s1 x s1)
    ;;(v "")
;;;; first direction = direction from first point of {x}-C towards origin
    ;;(!v- v s1)
    ;;(v "v->~s~%" v)
    ;; 2nd point = support in first direction
    ;;(support-function trial:primitive v s1)
    ;;(!v- s1 x s1)
    ;;(v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
    ;;(v<- v s1)
    (v "init dir = ~s~%" v)
    (loop for i from 0 below GJK-ITERATIONS
          while (progn
                  (v "~%iteration ~s/~s: (dim~s)~%" i gjk-iterations dim)
                  (v "v = ~s~%" v)
                  (v "?(<= ~s ~s) ~s~%"
                     (* 0.000001f0 maxdist) (vsqrlength v)
                     (<= (* 0.000001f0 maxdist) (vsqrlength v))
                     )
                  (<= (* 0.000001f0 maxdist) (vsqrlength v)))
          do (support-function trial:primitive v p)
             (!v- w x p)
             (v<- s0 w)
             (v "w=~s~%x=~s~%p=~s~%" w x p)
             (let ((vw (v. v w))
                   (vr (v. v r)))
               (v ":vw=~s, vr=~s~%" vw vr)
               (cond ((<= vw 0))
                     ((<= 0 vr)
                      (v "< 0 vr ~s~%" vr)
                      (return NIL))
                     (T
                      (let ((dt (/ vw vr)))
                        (decf λ dt)
                        (!v* dx r dt))
                      (!v+* x s r λ)
                      (v<- ray-normal v)
                      ;; update s[0-3] for new X
                      (!v- s0 s0 dx)
                      (!v- s1 s1 dx) ;; always have at least 2 points
                      ;; todo: don't calculate if we don't have that many
                      (!v- s2 s2 dx)
                      (!v- s3 s3 dx)
                      ;; reset duplicate point check
                      (setf stuck nil)
                      (v "λ=>~s~% x=~s~% ray-normal=~s~% dx=~s~%"
                         λ x ray-normal dx)
                      (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)))
               ;; update the simplex and find new direction
               (incf dim)
               (flet ((md (a b &optional c)
                        (let ((d (vsqrlength a))
                              (r a))
                          (when (> d (vsqrlength b))
                            (setf d (when c (vsqrlength b))
                                  r b))
                          (when (and c (> d (vsqrlength c)))
                            (setf r c))
                          r)))
                 (cond
                   ((= 1 dim)
                    (v<- s1 s0)
                    (!v- v s0))
                   ((= 2 dim)
                    (let* ((ao (v- s0))
                           (ab (v- s1 s0))
                           (ao.ab (v. ao ab)))
                      (v "ao=~s,ab=~s,.=~s~%" ao ab (v. ao ab))
                      (cond
                        ((< (vsqrlength ab) 0.00001)
                         (v "stuck2")
                         (cond
                           (stuck
                            (return nil))
                           (t
                            (setf stuck t)
                            (decf dim)
                            (v<- v ao))))
                        ((< 0 ao.ab)
                         ;;(!vc v (!vc v ab ao) ab)
                         (v<- s2 s1)
                         (v<- s1 s0)
                                        ;(v<- v (md s1 s2))
                         ;;(!v- v (md s1 s2))
                         
                         ;;(!v+* v s0 ab (/ ao.ab (v. ab ab)))
                         (!vc v (nvunit* (!vc v ab ao)) ab)
                         (v " v=>~s~%" v)
                         )
                        (t
                         (setf dim 1)
                         (v<- s1 s0)
                         (v<- v ao)))))
                   ((> 0.00001 (vsqrdistance s0 s1))
                    (v "stuck~aa~%" dim)
                    (when stuck
                      (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
                      (return nil))
                    (setf stuck t)
                    (decf dim)
                    (!v- v (md s1 s2 (when (= dim 4) s3))))
                   ((> 0.00001 (vsqrdistance s0 s2))
                    (v "stuck~ab~%" dim)
                    (when stuck
                      (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
                      (return nil))
                    (setf stuck t)
                    (decf dim)
                    (!v- v (md s1 s2 (when (= dim 4) s3))))
                   ((= 3 dim)
                    (setf dim (update-simplex s0 s1 s2 s3 v))
                                        ;(!v- v (md s1 s2 s3))
                    )
                   ((> 0.00001 (vsqrdistance s0 s3))
                    (v "stuck~ac~%" dim)
                    (when stuck
                      (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
                      (return nil))
                    (setf stuck t)
                    (decf dim)
                    (!v- v (md s1 s2 s3)))
                   ((null (test-simplex s0 s1 s2 s3 v))
                    (setf dim 3)
                                        ;(!v- v (md s1 s2 s3))
                    )
                   (T
                    (break "???"))))
               (!v- s0 s0 s0)
               (v "dims=~s~%" dim)
               (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3))
          finally (progn
                    (v "---~%final: ~s@~s~% m= ~s~%"
                       (v+* ray-location ray-direction λ) λ
                       ray-normal)
                    (nvunit* ray-normal)
                    (return λ)))))
(defun map-convex-mesh-faces (|(f v1 v2 v3)| primitive)
  (let ((verts (trial:convex-mesh-vertices primitive))
        (faces (trial:convex-mesh-faces primitive))
        (v1 (vec3))
        (v2 (vec3))
        (v3 (vec3)))
    (declare (dynamic-extent v1 v2 v3))
    (loop for f from 0 below (length faces) by 3
          do (flet ((vert (v i)
                      (let ((i (* 3 i)))
                        (vsetf v
                               (aref verts (+ i 0))
                               (aref verts (+ i 1))
                               (aref verts (+ i 2))))
                      v))
               (funcall |(f v1 v2 v3)|
                        (vert v1 (aref faces (+ f 0)))
                        (vert v2 (aref faces (+ f 1)))
                        (vert v3 (aref faces (+ f 2))))))))

(defun ref (ray-start ray-dir o)
  (let* ((d most-positive-single-float) ;; closest tₙ seen so far
         (p (vec3))
         (n (vec3))
         (found nil)
         (ϵ 0.0001)
         (ϵ² (expt ϵ 2)))
    (v "ref ~s~%   + ~s~%" ray-start ray-dir)
    (map-convex-mesh-faces
     (lambda (a b c)
       (let* ((ab (v- b a))
              (ac (v- c a))
              (ab×ac (vc ab ac))
              (-rd (v- ray-dir))
              (det (v. -rd ab×ac))
              (as (v- ray-start a))
              (t₁ (v. ab×ac as))
              ;;(t₂ (v. (vc ac -rd) ap))
              ;;(t₃ (v. (vc -rd ab) ap))
              )
         (v "test ~s,~s,~s~%" a b c)
         (v "  det = ~s, t₁ = ~s (~s)~%"
            det t₁ (unless (zerop det) (/ t₁ det)))
         (cond
           ;; ray is (almost) parallel to or on plane of triangle, test edges
           ((< (abs det) ϵ)
            (when (< (abs t₁) ϵ) ;; distance from ray-start to plane
              (flet ((edge (a ab)
                       (v "  edge ~s - ~s~%" a ab)
                       (let* ((as (v- ray-dir ab))
                              (n₁ (vc ray-dir ab))
                              (n₂ (vc ab n))
                              ;; line 1 = ray-start + t₁*ray-dir/d₁
                              (t₁ (v. as n₂))
                              (d₁ (v. ray-dir n₂))
                              ;; line 2 = a + t₂*ab/d₂
                              (t₂ (- (v. as n₁)))
                              (d₂ (v. ab n₁)))
                         (when (and
                                ;; lines are not parallel
                                (> (abs d₁) ϵ)
                                (> (abs d₂) ϵ)
                                ;; closest point is in ray
                                (> t₁ (- ϵ))
                                ;; and in edge
                                (> (1+ ϵ) t₂ (- ϵ)))
                           (let* ((t₁/d₁ (/ t₁ d₁))
                                  (p₁ (v+* ray-start ray-dir t₁/d₁))
                                  (p₂ (v+* a ab (/ t₂ d₂))))
                             (when (and
                                    ;;points are (nearly) same point
                                    (< (vsqrdistance p₁ p₂) ϵ²)
                                    ;; and closest point so far
                                    (< t₁/d₁ d))
                               (v "!!~%")
                               (setf found t
                                     d t₁/d₁)
                               (v<- p p₁)
                               (!vc n (!vc n ab ray-dir) ab)))))))
                (edge a ab)
                (edge a ac)
                (edge b (v- c b)))))
           ;; ray intersects plane at ray-start + t₁*ray-dir
           ((< 0 (setf t₁ (/ t₁ det)))
            (let* ((p₀ (v+* ray-start ray-dir t₁))
                   (ap (v- p₀ a))
                   (l (vsqrlength ab×ac))
                   (λ₂ (/ (v. (vc ap ac) ab×ac) l))
                   (λ₃ (/ (v. (vc ab ap) ab×ac) l)))
              (v "tri λ=~s, ~s (~s) t₁=~s~% p₀=~s~% n=~s~% ap=~s~%"
                 λ₂ λ₃ (+ λ₂ λ₃) t₁
                 p₀ n ap)
              ;; test if point is in triangle
              (when (and (< (- ϵ) λ₂ (1+ ϵ))
                         (< (- ϵ) λ₃ (1+ ϵ))
                         (< (+ λ₂ λ₃) (1+ ϵ)))
                (when (< t₁ d)
                  (v "!! @ ~s=~s n ~s~%" t₁ p₀ ab×ac)
                  (setf found t
                        d t₁)
                  (v<- p p₀)
                  (v<- n ab×ac)))))
           ;; intersects plane behind ray
           (t))))
     o)
    (when found
      (values p (nvunit n)))))

(defun pcopy (p)
  (let ((a (point)))
    (p<- a p)
    a))

(trial:define-ray-test trial:primitive ()
  (let* (;; inputs
         (s ray-location)
         (r ray-direction)
         ;; output n = ray-normal
         ;; state
         (λ 0.0)
         (x (vcopy s))
         (w (vec3))
         (p (vec3))
         (v (vcopy x))
         ;; amount we moved X, used to update cso
         ;;(dx (vec3))
         ;; set P
         (dim 0) ;; # of valid elements in P
         (s0 (point))
         (s1 (point))
         (s2 (point))
         (s3 (point))
         ;; stuff for alternative termination tests
         (stuck 0)
         (last-updated 0)
         ;; used to scale tolerance for termination test relative to simplex
         (maxdist 1.0)
         ;; 0.000003f0 is too small, loops with no progress on some
         ;; inputs
         ;; s-f-e * 240
         (ϵ (* 3 single-float-epsilon))
         )
    (declare (dynamic-extent λ x v w p s0 s1 s2 s3))
    (declare (type (unsigned-byte 8) dim stuck)
             (type point s0 s1 s2 s3)
             (type single-float maxdist))

    (vsetf ray-normal 0 0 0)
    (v "~&rloc =~s dir=~s~% p=~s~% v=~s~% x=~s~%"
       ray-location ray-direction p v x)
    (v "init dir = ~s~%" v)
    (loop for i from 0 below GJK-ITERATIONS
          do (v "~%iteration ~s/~s: (dim~s)~%" i gjk-iterations dim)
             (v "?(<= ~s ~s) ~s~%"
                (* ϵ maxdist) (vsqrlength v)
                (<= (* ϵ maxdist) (vsqrlength v)))
                                        ;while (<= (* ϵ maxdist) (vsqrlength v))
          do ;; we test v after updating λ since a final adjustment to
             ;; that improves results slightly. but we need to use the
             ;; values corresponding to v to determine cutoff
             ;; tolerance, so calculate that here
             (setf maxdist (max (if (> dim 0) (vsqrlength s0) 0.0)
                                (if (> dim 1) (vsqrlength s1) 0.0)
                                (if (> dim 2) (vsqrlength s2) 0.0)
                                (if (> dim 3) (vsqrlength s3) 0.0)))
             (d nil nil)
             (d :obj trial:primitive)
             (d :v1 v)
             (support-function trial:primitive v p)
             (d :p p)
             (d :x1 x)
             (d :rl ray-location)
             (d :rd ray-direction)
             (!v- w x p)
             (v<- (point-a s0) p)
             (v<- s0 w)
             (v "x=~s~%" x)
             (v "p=~s~%" p)
             (v "v=~s~%" v)
             (v "w=~s~%" w)
             (v "r=~s~%" r)
             (let ((vw (v. v w))
                   (vr (v. v r)))
               (d :vw w)
               (d :vr r)
               (v ":vw=~s, vr=~s (λ=~s)~%"
                  vw vr (unless (zerop vr) (- (/ vw vr))))
               (cond ((<= vw 0)
                      #++(<= vw 0 0.0000001)) ;; 0.00001 is too high, 0 is too low
                     ((<= 0 vr)
                      ;;(<= -0.00001 vr)
                      (v "< 0 vr ~s~%" vr)
                      (return NIL))
                     (T
                      (let ((dt (/ vw vr)))
                        (if (= (- λ dt) λ)
                            ;; if we failed to advance, we are
                            ;; probably stuck, so give up after a few
                            ;; tries
                            (incf stuck)
                            (setf stuck 0))
                        (decf λ dt)
                        (v "dt =~s, dx = ~s | stuck ~s~%" dt (v* r dt) stuck))
                      (!v+* x s r λ)
                      (v<- ray-normal v)
                      ;; clear for debugging clarity
                      (v<- s1 0)
                      (v<- s2 0)
                      (v<- s3 0)
                      (setf last-updated i)
                      ;; update s[0-3] for new X
                      (!v- s0 x (point-a s0))
                      (when (< 0 dim)
                        (!v- s1 x (point-a s1))
                        (when (< 1 dim)
                          (!v- s2 x (point-a s2))
                          (when (< 2 dim)
                            (!v- s3 x (point-a s3)))))
                      (d :x2 x)
                      (v "λ=>~s~% x=~s~% ray-normal=~s~%"
                         λ x ray-normal)
                      (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
                      (v "  s0a=~s~%  s1a=~s~%  s2a=~s~%  s3a=~s~%"
                         (point-a s0) (point-a s1)
                         (point-a s2) (point-a s3)))))
             ;; update the simplex and find new direction
             (d :x x)
             (incf dim)
             (cond
               ((and (< 1 dim) (v= (point-a s0) (point-a s1)))
                (v " remove duplicate s1=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                #++(unless (= last-updated i)
                     ;; this possible means we missed? need to see if it
                     ;; triggers in other cases or not
                     (error "repeated same point in simplex (i=~s,last=~s)"
                            i last-updated))
                (decf dim)
                (p<- s1 s2)
                (p<- s2 s3)
                (v<- s3 0))
               ((and (< 2 dim) (v= (point-a s0) (point-a s2)))
                (v " remove duplicate s2=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                (decf dim)
                (p<- s2 s3)
                (v<- s3 0))
               ((and (< 3 dim) (v= (point-a s0) (point-a s3)))
                (v " remove duplicate s3=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                (decf dim)
                (v<- s3 0)))

             (locally (declare (optimize (speed 1)))
               (d :s (mapcar 'pcopy (ecase dim
                                        ;(0 nil)
                                      (1 (list s0))
                                      (2 (list s0 s1))
                                      (3 (list s0 s1 s2))
                                      (4 (list s0 s1 s2 s3))))))
             (when (and *steps* (eql (* 2 i) *steps*))
               (v "  s0=~s=~s~%  s1=~s=~s~%  s2=~s=~s~%  s3=~s=~s~%"
                  s0 (when (= dim 4) (v- x s0))
                  s1 (v- x s1)
                  s2 (when (< 1 dim) (v- x s2))
                  s3 (when (< 2 dim) (v- x s3)))
               (v "v = ~s~%" v)
               (v "?(<= ~s ~s) ~s~%"
                  (* ϵ maxdist) (vsqrlength v)
                  (<= (* ϵ maxdist) (vsqrlength v)))
               
               (v "?(<= ~s ~s) ~s~%"
                  (* ϵ maxdist) (vsqrlength v)
                  (<= (* ϵ maxdist) (vsqrlength v)))
               (throw :step :step))

                                        ;while (<= (* ϵ maxdist) (vsqrlength v))
          while (progn
                  (unless (<= (* ϵ maxdist) (vsqrlength v))
                    (v "end @ v²=~s > ~s (~s), λ=~s~%" (vsqrlength v)
                       (* ϵ maxdist) maxdist λ))
                  (<= (* ϵ maxdist) (vsqrlength v)))
          until (> stuck 3)
          do
             (setf dim (signed-volumes dim s0 s1 s2 s3 v))
             (d :v2 v)
             (unless (= dim 4)
               (v<- s0 0))
             (v "dims=~s~%" dim)
                                        ;(v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
             (v "  s0=~s=~s~%  s1=~s=~s~%  s2=~s=~s~%  s3=~s=~s~%"
                s0 (when (= dim 4) (v- x s0))
                s1 (v- x s1)
                s2 (when (< 1 dim) (v- x s2))
                s3 (when (< 2 dim) (v- x s3)))
             (v "  s0a=~s~%  s1a=~s~%  s2a=~s~%  s3a=~s~%"
                (point-a s0) (point-a s1)
                (point-a s2) (point-a s3))
             (locally (declare (optimize (speed 1)))
               (d :s (mapcar 'pcopy (ecase dim
                                      (0 nil)
                                      (1 (list s1))
                                      (2 (list s1 s2))
                                      (3 (list s1 s2 s3))
                                      (4 (list s0 s1 s2 s3))))))

          when (and *steps* (eql (1+ (* 2 i)) *steps*))
            do (v "v = ~s~%" v)
               (v "?(<= ~s ~s) ~s~%"
                  (* ϵ maxdist) (vsqrlength v)
                  (<= (* ϵ maxdist) (vsqrlength v)))
               (throw :step :step)
          finally (progn
                    (d :steps i)
                    (v "---~%final: ~s@~s (~sit)~% m= ~s~%"
                       (v+* ray-location ray-direction λ) λ i
                       ray-normal)
                    (nvunit* ray-normal)
                    (return λ)))))

(defun update-simplex (s0 s1 s2 s3 dir)
  ;(declare (optimize speed (safety 0)))
  (declare (type point s0 s1 s2 s3))
  (declare (type vec3 dir))
  (let ((n (vec3)) (ao (v- s0)))
    (declare (dynamic-extent n ao))
    (!vc n (v- s1 s0) (v- s2 s0))
    (v "us: n=~s ao=~s~% 1=~s ~s~% 2=~s ~s~% 3=~s~%"
       n ao
       (vc (v- s1 s0) n) (v. ao (vc (v- s1 s0) n))
       (vc n (v- s2 s0)) (v. ao (vc n (v- s2 s0)))
       (v. n ao))
    (v "  s0=~s~%  s1=~s~%  s2=~s~%  s3=~s~%" s0 s1 s2 s3)
    (cond ((< 0 (v. ao (vc (v- s1 s0) n)))
           (v "1~%")
           (p<- s2 s0)
           (!vc dir (!vc dir (v- s1 s0) ao) (v- s1 s0))
           2)
          ((< 0 (v. ao (vc n (v- s2 s0))))
           (v "2~%")
           (p<- s1 s0)
           (!vc dir (!vc dir (v- s2 s0) ao) (v- s2 s0))
           2)
          ((< 0 (v. n ao))
           (v "3~%")
           (p<- s3 s2)
           (p<- s2 s1)
           (p<- s1 s0)
           (v<- dir n)
           3)
          (T
           (v "4~%")
           (p<- s3 s1)
           (p<- s1 s0)
           (v<- dir n)
           (nv- dir)
           3))))

(defun test-simplex (s0 s1 s2 s3 dir)
  (declare (optimize speed (safety 0)))
  (declare (type point s0 s1 s2 s3))
  (declare (type vec3 dir))
  (let ((abc (vec3)) (acd (vec3)) (adb (vec3)) (ao (v- s0)))
    (declare (dynamic-extent abc acd adb ao))
    (!vc abc (v- s1 s0) (v- s2 s0))
    (!vc acd (v- s2 s0) (v- s3 s0))
    (!vc adb (v- s3 s0) (v- s1 s0))
    (cond ((< 0 (v. abc ao))
           (p<- s3 s2)
           (p<- s2 s1)
           (p<- s1 s0)
           (v<- dir abc)
           NIL)
          ((< 0 (v. acd ao))
           (p<- s1 s0)
           (v<- dir acd)
           NIL)
          ((< 0 (v. adb ao))
           (p<- s2 s3)
           (p<- s3 s1)
           (p<- s1 s0)
           (v<- dir adb)
           NIL)
          (T
           T))))

;;;; EPA for depth and normal computation
;;; FIXME: stack allocation bullshit
(defun epa (s0 s1 s2 s3 a b hit)
  (declare (optimize speed (safety 1)))
  (declare (type point s0 s1 s2 s3))
  (declare (type trial:hit hit))
  (let ((faces (make-array (* 4 EPA-MAX-FACES)))
        (loose-edges (make-array (* 2 EPA-MAX-LOOSE-EDGES)))
        (num-faces 4) (closest-face 0) (min-dist 0.0)
        (search-dir (vec3)) (p (point)))
    (declare (dynamic-extent faces loose-edges search-dir p))
    (declare (type (unsigned-byte 16) num-faces))
    (dotimes (i (* 4 EPA-MAX-FACES))
      (setf (aref faces i) p)
      (setq p (point)))
    (dotimes (i (* 2 EPA-MAX-LOOSE-EDGES))
      (setf (aref loose-edges i) p)
      (setq p (point)))
    (macrolet ((v (f v)
                 `(the point (aref faces (+ (* 4 ,f) ,v))))
               (e (e v)
                 `(the point (aref loose-edges (+ (* 2 ,e) ,v)))))
      ;; Construct the initial polytope
      ;; The FACES array contains the packed vertices and normal of each face
      (p<- (v 0 0) s0)
      (p<- (v 0 1) s1)
      (p<- (v 0 2) s2)
      (plane-normal s0 s1 s2 (v 0 3))
      (p<- (v 1 0) s0)
      (p<- (v 1 1) s2)
      (p<- (v 1 2) s3)
      (plane-normal s0 s2 s3 (v 1 3))
      (p<- (v 2 0) s0)
      (p<- (v 2 1) s3)
      (p<- (v 2 2) s1)
      (plane-normal s0 s3 s1 (v 2 3))
      (p<- (v 3 0) s1)
      (p<- (v 3 1) s3)
      (p<- (v 3 2) s2)
      (plane-normal s1 s3 s2 (v 3 3))
      ;; Main iteration loop to find the involved faces
      (dotimes (i EPA-ITERATIONS)
        ;; Find the closest face in our set of known polytope faces
        (setf min-dist (v. (v 0 0) (v 0 3)))
        (setf closest-face 0)
        (loop for i from 1 below num-faces
              for dist = (v. (v i 0) (v i 3))
              do (when (< dist min-dist)
                   (setf min-dist dist)
                   (setf closest-face i)))
        (v<- search-dir (v closest-face 3))
        ;; Find a new direction to search in via the support functions
        (search-point p search-dir a b)
        (when (< (- (v. p search-dir) min-dist) EPA-TOLERANCE)
          (return))
        ;; We still haven't found a face that's good enough, so expand the
        ;; polytope from our current face set
        (let ((num-loose-edges 0)
              (i 0))
          (declare (type (unsigned-byte 16) num-loose-edges i))
          ;; Find triangles facing our current search point
          (loop (when (<= num-faces i) (return))
                (cond ((< 0 (v. (v i 3) (!v- search-dir p (v i 0))))
                       ;; ... I'm not sure how this part works, exactly.
                       ;; It manages the loose edge list to expand the polytope?
                       (loop for j from 0 below 3
                             for edge-a = (v i j)
                             for edge-b = (v i (mod (1+ j) 3))
                             for edge-found-p = NIL
                             do (dotimes (k num-loose-edges)
                                  (when (and (v= (e k 1) edge-a)
                                             (v= (e k 0) edge-b))
                                    (decf num-loose-edges)
                                    (p<- (e k 0) (e num-loose-edges 0))
                                    (p<- (e k 1) (e num-loose-edges 1))
                                    (setf edge-found-p T)
                                    (return)))
                                (unless edge-found-p
                                  (when (<= EPA-MAX-LOOSE-EDGES num-loose-edges)
                                    (return))
                                  (p<- (e num-loose-edges 0) edge-a)
                                  (p<- (e num-loose-edges 1) edge-b)
                                  (incf num-loose-edges)))
                       ;; This face is no longer facing our search point, so we remove
                       ;; it by replacing it with the tail face
                       (decf num-faces)
                       (p<- (v i 0) (v num-faces 0))
                       (p<- (v i 1) (v num-faces 1))
                       (p<- (v i 2) (v num-faces 2))
                       (v<- (v i 3) (v num-faces 3)))
                      (T
                       (incf i))))
          ;; Expand the polytope with the search point added to the new loose edge faces
          (dotimes (i num-loose-edges)
            (when (<= EPA-MAX-FACES num-faces)
              (return))
            (p<- (v num-faces 0) (e i 0))
            (p<- (v num-faces 1) (e i 1))
            (p<- (v num-faces 2) p)
            (plane-normal (v num-faces 0) (v num-faces 1) (v num-faces 2) (v num-faces 3))
            ;; Check the CCW winding order via normal test
            (when (< (+ (v. (v num-faces 0) (v num-faces 3)) 0.000001) 0)
              (rotatef (v num-faces 0) (v num-faces 1))
              (nv- (v num-faces 3)))
            (incf num-faces))))

      ;; Compute the actual intersection
      ;; If we did not converge, we just use the closest face we reached
      (%epa-finish hit (v closest-face 0) (v closest-face 1) (v closest-face 2)))))

(defun %epa-finish (hit c0 c1 c2)
  (declare (type point c0 c1 c2))
  (declare (type trial:hit hit))
  (declare (optimize speed (safety 0)))
  (let ((p (vec3)) (a-point (vec3)) (b-point (vec3)))
    (declare (dynamic-extent p a-point b-point))
    (barycentric c0 c1 c2 (plane-point c0 c1 c2 p) p)
    (nv+* a-point (point-a c0) (vx p))
    (nv+* a-point (point-a c1) (vy p))
    (nv+* a-point (point-a c2) (vz p))
    (nv+* b-point (point-b c0) (vx p))
    (nv+* b-point (point-b c1) (vy p))
    (nv+* b-point (point-b c2) (vz p))
    (v<- (trial:hit-location hit) a-point)
    (v<- (trial:hit-normal hit) b-point)
    (nv- (trial:hit-normal hit) a-point)
    (setf (trial:hit-depth hit) (vlength (trial:hit-normal hit)))
    (if (= 0.0 (trial:hit-depth hit))
        (v<- (trial:hit-normal hit) +vy3+)
        (nv/ (trial:hit-normal hit) (trial:hit-depth hit)))))

;;;; Support function implementations
(defun %support-function (primitive global-direction next)
  (declare (optimize speed (safety 0)))
  (declare (type trial:primitive primitive))
  (declare (type vec3 global-direction next))
  (let ((local (vcopy global-direction)))
    (declare (dynamic-extent local))
    (trial::ntransform-inverse local (trial:primitive-transform primitive))
    (support-function primitive local next)
    (n*m (trial:primitive-transform primitive) next)))

(defgeneric support-function (primitive local-direction next))

(defmacro define-support-function (type (dir next) &body body)
  `(defmethod support-function ((primitive ,type) ,dir ,next)
     (declare (type vec3 ,dir ,next))
     (declare (optimize speed))
     ,@body))

(define-support-function trial:plane (dir next)
  (let ((denom (v. (trial:plane-normal primitive) dir)))
    (vsetf next 0 0 0)
    (if (<= denom 0.000001)
        (nv+* next dir (trial:plane-offset primitive))
        (let ((tt (/ (trial:plane-offset primitive) denom)))
          (nv+* next dir tt)))))

(define-support-function trial:sphere (dir next)
  (nv* (nvunit* (v<- next dir)) (trial:sphere-radius primitive)))

(define-support-function trial:box (dir next)
  (let ((bsize (trial:box-bsize primitive)))
    (vsetf next
           (if (< 0 (vx3 dir)) (vx3 bsize) (- (vx3 bsize)))
           (if (< 0 (vy3 dir)) (vy3 bsize) (- (vy3 bsize)))
           (if (< 0 (vz3 dir)) (vz3 bsize) (- (vz3 bsize))))))

(define-support-function trial:pill (dir next)
  (nv* (nvunit* (v<- next dir)) (trial:pill-radius primitive))
  (let ((bias (trial:pill-height primitive)))
    (if (< 0 (vy dir))
        (incf (vy next) bias)
        (decf (vy next) bias))))

(define-support-function trial:cylinder (dir next)
  (vsetf next (vx dir) 0 (vz dir))
  (nv* (nvunit* next) (trial:cylinder-radius primitive))
  (if (< 0 (vy dir))
      (incf (vy next) (trial:cylinder-height primitive))
      (decf (vy next) (trial:cylinder-height primitive))))

(define-support-function trial:triangle (dir next)
  (let ((furthest most-negative-single-float))
    (flet ((test (vert)
             (let ((dist (v. vert dir)))
               (when (< furthest dist)
                 (setf furthest dist)
                 (v<- next vert)))))
      (test (trial:triangle-a primitive))
      (test (trial:triangle-b primitive))
      (test (trial:triangle-c primitive)))))

(define-support-function trial:convex-mesh (dir next)
  (let ((verts (trial:convex-mesh-vertices primitive))
        (vert (vec3))
        (furthest most-negative-single-float))
    (declare (dynamic-extent vert))
    ;; FIXME: this is O(n)
    (loop for i from 0 below (length verts) by 3
          do (vsetf vert
                    (aref verts (+ i 0))
                    (aref verts (+ i 1))
                    (aref verts (+ i 2)))
             (let ((dist (v. vert dir)))
               #++(v "*~s @ ~s" vert dist)
               (when (< furthest dist)
                 (setf furthest dist)
                 (v<- next vert)
                 #++(v "!!"))
               #++(v "~%")))
    (v "* -> ~s @ ~s~%" next furthest)))

