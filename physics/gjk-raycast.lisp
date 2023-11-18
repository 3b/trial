;;; The raycast implementation is largely based on "Ray Casting
;;; against General Convex Objects with Application to Continuous
;;; Collision Detection" by Gino Van Den Bergen available at
;;; http://dtecta.com/papers/jgt04raycast.pdf

;;; Raycasting uses an alternative implementation of GJK originally
;;; based on "Improving the GJK algorithm for faster and more reliable
;;; distance queries between convex objects." by Mattia Montanari, Nik
;;; Petrinic, and Ettore Barbieri. 2017.  ACM Trans. Graph. 36, 3,
;;; Article 30 (June 2017) DOI: http://dx.doi.org/10.1145/3083724

;;; Probably could be made to share more code with the main GJK
;;; implementation, but would probably slow down the normal path, so
;;; didn't bother trying. In particular, the raycast algorithm depends
;;; on knowing the nearest point on the current simplex, rather than
;;; just the direction to that point.

(in-package #:org.shirakumo.fraf.trial.gjk)

;; todo: define a separate POINT type to use for raycast, to avoid
;; extra copies of unused B slot in P<-
(declaim (inline pcopy))
(defun pcopy (p)
  (let ((a (point)))
    (p<- a p)
    a))

(defvar *steps* nil)
(defvar *f* nil)

(defun .v (f &rest r)
  (when *f*
    (apply 'format t f r)))

(defmacro v (&rest r)
  `(when *f* (.v ,@r)))

(defparameter *debug-state* (make-hash-table))

(defun .d (k v)
  (when *debug-state*
    (if (eql k nil)
        (clrhash *debug-state*)
        (setf (gethash k *debug-state*)
              (if (vec-p v) (vcopy v) v)))))

(defmacro d (&rest r)
  `(when *debug-state* (.d ,@r)))


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
       (v<- dir s1)
       1)
      (2 (sv1d s0 s1 s2 s3 dir))
      (3 (sv2d s0 s1 s2 s3 dir))
      (4 (sv3d s0 s1 s2 s3 dir)))))

(declaim (inline sv-compare-signs))
(defun sv-compare-signs (a b)
  (or (and (< a 0) (< b 0))
      (and (< 0 a) (< 0 b))))

(defun sv3d (s0 s1 s2 s3 dir)
  (declare (type point s0 s1 s2 s3)
           (type vec3 dir)
           (optimize speed))
  (let* ((m (mat4 (vx s0) (vx s1) (vx s2) (vx s3)
                  (vy s0) (vy s1) (vy s2) (vy s3)
                  (vz s0) (vz s1) (vz s2) (vz s3)
                  1 1 1 1))
         (c (vec4))
         (mdet 0f0)
         (flat nil)
         (ϵ 0.00001))
    (declare (dynamic-extent m c)
             (type single-float mdet))
    (loop for j below 4
          for cj = (mcofactor m 3 j)
          do (setf (vref c j) cj)
             (incf mdet cj))
    (setf flat (< (abs mdet) ϵ))
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
         (declare (dynamic-extent best-dir
                                  b0 b1 b2
                                  cdir c0 c1 c2 c3))
         (v " faces det~s, c~s (flat ~s)~%" mdet c flat)
         (loop for j from 1 to 3
               when (or flat
                        (not (sv-compare-signs mdet (vref c j))))
                 do (p<- c0 (if (= j 0) s1 s0))
                    (p<- c1 (if (< j 2) s2 s1))
                    (p<- c2 (if (< j 3) s3 s2))
                    (let ((r (sv2d c0 c1 c2 c3 cdir))
                          (d* (vsqrlength cdir)))
                      (declare (type (unsigned-byte 4) r))
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

(declaim (inline projected-cross v2c))
(defun projected-cross (a b x)
  ;; project A,B onto plane normal to axis with index X, then return
  ;; X'th component of resulting vectors
  (macrolet ((c (x y)
               `(- (* (,x a) (,y b))
                   (* (,y a) (,x b)))))
    (ecase x
      (0 (c vy vz))
      (1 (- (c vx vz)))
      (2 (c vx vy)))))

(defun v2c (a b)
  (- (* (vx a) (vy b))
     (* (vy a) (vx b))))

(defun sv2d (s0 s1 s2 s3 dir)
  (declare (type point s0 s1 s2 s3)
           (type vec3 dir)
           (optimize speed))
  (let* ((ab (v- s1 s0))
         (ac (v- s2 s0))
         (bc (v- s2 s1))
         ;; vc doesn't get DX yet?
         ;;(n (vc ac ab))
         ;;(l² (vsqrlength n))
         (n (vec3))
         (l² 0.0)
         (p₀ (vec3))
         (μₘₐₓ 0.0)
         (j -1)
         (flat nil)
         (ϵ 0.00001))
    (declare (dynamic-extent ab ac bc n p₀)
             (type single-float μₘₐₓ))
    (!vc n ac ab)
    (setf l² (vsqrlength n))
    (v "2d:~%")
    (v "  s0=~s~%  s1=~s~%  s2=~s~%" s0 s1 s2)
    ;; project origin onto plane (if possible)
    (if (< l² (expt ϵ 2))
        (setf flat t)
        (!v* p₀ n (/ (v. s0 n) (vsqrlength n))))
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
        (declare (dynamic-extent best-dir
                                 b0 b1
                                 cdir c0 c1 c2 c3))
        (v " edges~%")
        (loop for j from 0 to 2
              do (p<- c0 (if (= j 0) s1 s0))
                 (p<- c1 (if (< j 2) s2 s1))
                 (let ((r (sv1d c0 c1 c2 c3 cdir))
                       (d* (vsqrlength cdir)))
                   (declare (type (unsigned-byte 4) r))
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
          for μ of-type single-float = (projected-cross ab ac i)
          do (v " i=~s,μ=~s~a~%" i μ (if (> (abs μ) (abs μₘₐₓ)) "!!" ""))
          when (> (abs μ) (abs μₘₐₓ))
            do (setf μₘₐₓ μ
                     j i))
    (setf flat (< (abs μₘₐₓ) ϵ))
    (v " j=~s, flat=~s~%" j flat)
    (assert (not flat))

    ;; calculate barycentric coordinates on that plane (seems slightly
    ;; more accurate than calling BARYCENTRIC, and not sure that
    ;; handles points outside the triangle the way we want?
    (let* ((ap (v- p₀ s0))
           (pb (v- p₀ s1))
           (pc (v- p₀ s2))
           (sabc (projected-cross ab ac j))
           (s (if (minusp sabc) -1 1))
           (abc (abs sabc))
           (λ₁ (* s (projected-cross pb pc j)))
           (n₁ (<= λ₁ 0))
           (λ₂ (* s (projected-cross ap ac j)))
           (n₂ (<= λ₂ 0))
           (λ₃ (* s (projected-cross ab ap j)))
           (n₃ (<= λ₃ 0))
           (out (+ (if n₁ 1 0) (if n₂ 1 0) (if n₃ 1 0))))
      (declare (dynamic-extent ap pb pc))
      (v "  λ = ~s,~s,~s / ~s = ~s~%" λ₁ λ₂ λ₃ abc (/ (+ λ₁ λ₂ λ₃) abc))
      (cond
        ;; support is entire triangle
        ((and (zerop out)
              (< λ₁ abc)
              (< λ₂ abc)
              (< λ₃ abc))
         ;; possibly should recalculate one of the coords from the
         ;; other 2, but not sure which.
         (!v* dir s0 (/ λ₁ abc))
         (!v+* dir dir s1 (/ λ₂ abc))
         (!v+* dir dir s2 (/ λ₃ abc))
         (p<- s3 s2)
         (p<- s2 s1)
         (p<- s1 s0)
         (v "  3dir<-~s~%" dir)
         (v "   s1=~s~%   s2=~s~%   s3=~s~%" s1 s2 s3)
         3)
        ((= out 1)
         ;; support is line opposite negative λ
         (flet ((edge (a b m)
                  (let* ((t₀ (- (/ (v. m a)
                                   (v. m m)))))
                    (v " a=~s~% b=~s~% m=~s~% t₀=~s~%" a b m t₀)
                    (!v+* dir a m t₀))
                  (p<- s2 b)
                  (p<- s1 a)
                  (v "  3dir2<-~s~%" dir)
                  (v "   s1=~s~%   s2=~s~%" s1 s2)
                  2))
           (cond
             (n₁ (edge s1 s2 bc))
             (n₂ (edge s0 s2 ac))
             (n₃ (edge s0 s1 ab)))))
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
        (t (error "shouldn't get here?"))))))

(defun sv1d (s0 s1 s2 s3 dir)
  (declare (type point s0 s1 s2 s3)
           (type vec3 dir)
           (optimize speed))
  (declare (ignore s3))
  (v "1d:~%")
  (v "  s0=~s~%  s1=~s~%" s0 s1)
  (let* ((m (v- s1 s0))
         (mm (v. m m))
         (ϵ 0.00001))
    (declare (dynamic-extent m))
    (v "  mm=~s,m=~s~%" mm m)
    (cond
      ((< (abs mm) ϵ)
       ;; degenerate segment, return either point (s1 to avoid a copy)
       (v " dir=s0=~s~%" s1)
       (v<- dir s1)
       1)
      (t
       (let* ((t₀ (- (/ (v. m s0)
                        (v. m m)))))
         (v " a=~s~% b=~s~% t₀=~s~%" s0 s1 t₀)
         (cond
           ((<= t₀ 0)
            ;; start point (keep s0 in s1)
            (v " dir1a=s0=~s~%" s0)
            (v<- dir s0)
            (p<- s1 s0)
            1)
           ((<= 1 t₀)
            ;; end point (keep s1 in s1)
            (v " dir1b=s0=~s~%" s1)
            (v<- dir s1)
            1)
           (t
            ;; keep both points (in s1,s2)
            (!v+* dir s0 m t₀)
            (p<- s2 s1)
            (p<- s1 s0)
            (v " 1dir2<-~s~%" dir)
            (v "   s1=~s~%   s2=~s~%" s1 s2)
            2)))))))

(trial:define-ray-test trial:primitive ()
  (let* (;; inputs
         (s (vcopy ray-location))
         (r ray-direction)
         (ray-offset 0.0)
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
         ;; paper says "order of magnitude larger than machine
         ;; epsilon" (* 6 single-float-negative-epsilon) seems like a
         ;; good balance between quality of results and chance of
         ;; getting stuck in a loop (8 is about half as likely as 6 to
         ;; loop, 5 is slightly more likely to loop, 4 is about 2x, 3
         ;; is ~4x, 2 is ~7x).
         ;; we mostly detect the loops though, so this just tunes tiny
         ;; performance difference vs tiny precision differences.
         (ϵ (* 6 single-float-negative-epsilon)))
    (declare (dynamic-extent λ x v w p s0 s1 s2 s3))
    (declare (type (unsigned-byte 8) dim stuck)
             (type point s0 s1 s2 s3)
             (type single-float maxdist))
    (vsetf ray-normal 0 0 0)
    (v "~&rloc = ~s~% (~s + ~s)~% dir=~s~% p=~s~% v=~s~% x=~s~%"
       s ray-location ray-offset ray-direction p v x)
    (v "init dir = ~s~%" v)
    (loop for i from 0 below GJK-ITERATIONS
          do (v "~%iteration ~s/~s: (dim~s)~%" i gjk-iterations dim)
             (v "?(<= ~s ~s) ~s~%"
                (* ϵ maxdist) (vsqrlength v)
                (<= (* ϵ maxdist) (vsqrlength v)))
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
             (d :rl1 ray-location)
             (d :rl s)
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
               (cond ((<= vw 0))
                     ((<= 0 vr)
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
             (d :x x)
             (incf dim)
             ;; if point we just added is already in simplex, drop old
             ;; copy since we depend on ordering of vertices and
             ;; degenerate simplexes confuse things anyway. It is
             ;; normal to see duplicates after moving the simplex,
             ;; though duplicates without making any progress tends to
             ;; be a sign we are stuck.
             (cond
               ((and (< 1 dim) (v= (point-a s0) (point-a s1)))
                (v " remove duplicate s1=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                (decf dim)
                (when (/= i last-updated)
                  (incf stuck))
                (p<- s1 s2)
                (p<- s2 s3)
                (v<- s3 0))
               ((and (< 2 dim) (v= (point-a s0) (point-a s2)))
                (v " remove duplicate s2=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                (decf dim)
                (when (/= i last-updated)
                  (incf stuck))
                (p<- s2 s3)
                (v<- s3 0))
               ((and (< 3 dim) (v= (point-a s0) (point-a s3)))
                (v " remove duplicate s3=s0 dim->~s (i=~s,last=~s)~%" (1- dim)
                   i last-updated)
                (decf dim)
                (when (/= i last-updated)
                  (incf stuck))
                (v<- s3 0)))

             (d :s (mapcar 'pcopy (ecase dim
                                        ;(0 nil)
                                    (1 (list s0))
                                    (2 (list s0 s1))
                                    (3 (list s0 s1 s2))
                                    (4 (list s0 s1 s2 s3))
                                    (5 (list s0 s1 s2 s3)))))
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
             ;; check if we are done
          while (progn
                  (unless (<= (* ϵ maxdist) (vsqrlength v))
                    (v "end @ v²=~s > ~s (~s), λ=~s~%" (vsqrlength v)
                       (* ϵ maxdist) maxdist λ))
                  (<= (* ϵ maxdist) (vsqrlength v)))
          until (> stuck 3)
          do
             ;; update the simplex and find new direction
             (setf dim (signed-volumes dim s0 s1 s2 s3 v))
             (d :v2 v)
             (unless (= dim 4)
               (v<- s0 0))
             (v "dims=~s~%" dim)
             (v "  s0=~s=~s~%  s1=~s=~s~%  s2=~s=~s~%  s3=~s=~s~%"
                s0 (when (= dim 4) (v- x s0))
                s1 (v- x s1)
                s2 (when (< 1 dim) (v- x s2))
                s3 (when (< 2 dim) (v- x s3)))
             (v "  s0a=~s~%  s1a=~s~%  s2a=~s~%  s3a=~s~%"
                (point-a s0) (point-a s1)
                (point-a s2) (point-a s3))
             (d :s (mapcar 'pcopy (ecase dim
                                    (0 nil)
                                    (1 (list s1))
                                    (2 (list s1 s2))
                                    (3 (list s1 s2 s3))
                                    (4 (list s0 s1 s2 s3))
                                    (5 (list s0 s1 s2 s3)))))
          when (and *steps* (eql (1+ (* 2 i)) *steps*))
            do (v "v = ~s~%" v)
               (v "?(<= ~s ~s) ~s~%"
                  (* ϵ maxdist) (vsqrlength v)
                  (<= (* ϵ maxdist) (vsqrlength v)))
               (throw :step :step)
          finally (let ((f (+ λ ray-offset)))
                    (d :steps i)
                    (v "---~%final: ~s@~s+~s (~sit)~% m= ~s~%"
                       (v+* ray-location ray-direction f)
                       λ ray-offset i
                       ray-normal)
                    (when (>= f 0)
                      (nvunit* ray-normal)
                      (return f))))))
