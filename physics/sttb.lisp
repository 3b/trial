;;; based on
;;; https://www.bepuentertainment.com/blog/2022/3/10/seeking-the-tootbird

(defpackage #:org.shirakumo.fraf.trial.sttb
  (:use #:cl #:org.shirakumo.fraf.math)
  (:import-from #:org.shirakumo.fraf.trial.gjk
                #:point #:point-a #:point-b #:p<-
                #:barycentric
                #:search-point
                ;; from gjk-raycast
                #:projected-cross)
  (:export
   #:detect-hits))

(in-package #:org.shirakumo.fraf.trial.sttb)

(defconstant STTB-ITERATIONS 64)
(defconstant STTB-TOLERANCE 0.0001)

(defun update-simplex (dim s0 s1 s2 s3 dir tootbird best)
  ;; returns (values new-dim done-flag new-best)
  (declare (type point s0 s1 s2 s3)
           (type vec3 dir tootbird)
           (type single-float best)
           (type (unsigned-byte 2) dim))
  (step-trace dim s1 s2 s3)
  (trace-line (vec3) dir)
  (trace-point tootbird)
  (trace-point s0)
  ;; update tootbird

  ;;  find signed distance from origin to plane defined by new point
  ;;  S0 and (unit) normal DIR.
  (unless (zerop best)
    (let ((d (v. s0 dir)))
      
      ;; If negative, origin is outside Minkowski difference, and we
      ;; are done (unless we want to find closest point, in which case
      ;; keep going, possibly switching to seeking origin instead of
      ;; tootbird. Neither seems to work with current termination
      ;; tests though, so just exit for now.)
      (cond
        ((minusp d)
         (return-from update-simplex (values dim -1 d)))
        #++
        ((minusp d)
         ;; if we are finding closest point in non-intersecting case,
         ;; move tootbird to origin and set 'best' to 0 so we stop
         ;; checking it.
         (vzero tootbird)
         (setf best 0.0))
        ;; if distance is 0, new tootbird is the new point, which is on
        ;; surface of Minkowski difference, so we are done
        ((< (abs d) (expt STTB-TOLERANCE 2))
         (when (Zerop dim) ;; make sure we have some result
           (p<- s1 s0)
           (setf dim 1))
         (return-from update-simplex (values dim 1 0.0)))
        ;; if it is closer than best tootbird, project origin onto the
        ;; plane to get new tootbird
        ((< (abs d) best)
         (setf best (abs d))
         (!v* tootbird dir d)))))

  ;; update simplex
  (case dim
    (0
     (p<- s1 s0)
     (setf dim 1))
    (1
     (when (and (zerop best) (v~= s0 s1))
       ;; got same point twice in a row while searching for origin, we
       ;; are done
       (return-from update-simplex (values dim -1 best)))
     (p<- s2 s0)
     (setf dim 2))
    (2
     (when (and (zerop best)
                (or (v~= s0 s1) (v~= s0 s2)))
       (return-from update-simplex (values dim -1 best)))
     (p<- s3 s0)
     (setf dim 3))
    (3
     ;; pick new simplex based on planes normal to old simplex and
     ;; through new vertex and one of old vertices (this could
     ;; possibly share some calculations with the "point nearest
     ;; tootbird" calculation below, but one of the s1,s2,s2 are
     ;; different by then so would take some bookkeeping to
     ;; determine which parts to keep for a given calculation)
     (let* ((ab (v- s2 s1))
            (ac (v- s3 s1))
            (an (v- s0 s1))
            (bn (v- s0 s2))
            (cn (v- s0 s3))
            (tn (v- s0 tootbird))
            (n (vec3)))
       (declare (dynamic-extent ab ac an bn cn n))
       (cond
         ;; if new point is too close to one of the old ones, just
         ;; replace that one
         ((= (vsqrlength an) 0.0)
          ;; if we see a duplicate while in no-intersection mode, we
          ;; probably won't make any more progress, so return current
          ;; simplex as nearest contact
          (when (zerop best)
            (return-from update-simplex
              (values dim -1 best)))
          (p<- s1 s0))
         ((= (vsqrlength bn) 0.0)
          (when (zerop best)
            (return-from update-simplex
              (values dim -1 best)))
          (p<- s2 s0))
         ((= (vsqrlength cn) 0.0)
          (when (zerop best)
            (return-from update-simplex
              (values dim -1 best)))
          (p<- s3 s0))
         ;; otherwise do plane tests to decide which to keep
         (T
          (!vc n ac ab)
          ;; calculate normals of plane perpendicular to old plane, and
          ;; through new point and one of old points
          (nvc an n)
          (nvc bn n)
          (nvc cn n)
          ;; then determine which side of each of those planes
          ;; tootbird is on
          (let* ((da (v. tn an))
                 (db (v. tn bn))
                 (dc (v. tn cn))
                 (f (logior (if (plusp da) 1 0)
                            (if (plusp db) 2 0)
                            (if (plusp dc) 4 0))))
            (case f
              ((#b001 #b101) ;; in front of a, behind b, replace c
               (p<- s3 s0))
              ((#b010 #b011) ;; in front of b, behind c, replace a
               (p<- s1 s0))
              ((#b100 #b110) ;; in front of c, behind a, replace b
               (p<- s2 s0))
              ;; ambiguous, pick smallest. Original just picked a
              ;; specific point in this case, but this seems to work a
              ;; bit better?
              ((#b000 #b111)
               (if (< da db)
                   (if (< dc da)
                       (p<- s3 s0)
                       (p<- s1 s0))
                   (if (< db dc)
                       (if (< da dc)
                           (p<- s1 s0)
                           (p<- s2 s0))
                       (p<- s3 s0)))))))))))

  (flet ((line (a b dir)
           (let* ((m (v- b a))
                  (dt (v- tootbird a))
                  (mm (v. m m)))
             (declare (dynamic-extent m dt))
             (cond
               ((< mm 0.00000001)
                ;; degenerate, treat as 1 point (keep newer, in hopes it
                ;; works better next pass)
                (p<- a b)
                (v<- dir a)
                1)
               (T
                (let* ((t0 (/ (v. m dt) mm)))
                  (cond
                    ((<= t0 0)
                     (v<- dir a)
                     1)
                    ((<= 1 t0)
                     (v<- dir b)
                     (p<- a b)
                     1)
                    (T
                     (!v+* dir a m t0)
                     2))))))))
    ;; find point on simplex nearest to tootbird (store it in dir)
    (case dim
      (1
       ;; if only 1 point, that is closest point
       (v<- dir s1))
      (2
       (setf dim (line s1 s2 dir)))
      (3
       (let* ((ab (v- s2 s1))
              (ac (v- s3 s1))
              (at (v- tootbird s1))
              (n (vec3))
              (l^2 0.0)
              (p0 (vec3))
              (umax 0.0)
              (j -1)
              (flat NIL)
              (epsilon 0.00001))
         (declare (dynamic-extent ab ac at n p0)
                  (type single-float umax))
         (!vc n ac ab)
         (setf l^2 (vsqrlength n))
         ;; project tootbird onto plane of simplex (if possible)
         (cond
           ((< l^2 (expt epsilon 2))
            (setf flat T))
           (T
            (!v* p0 n (/ (v. at n) l^2))
            (!v- p0 tootbird p0)))

         ;; if too flat, just pick best result from 1d test against all edges
         (flet ((flat ()
                  (let ((d MOST-POSITIVE-SINGLE-FLOAT)
                        ;; best result seen so far
                        (best-dim 0)
                        (best-dir (vec3))
                        (b0 (point))
                        (b1 (point))
                        ;; temp space used by each LINE call
                        (cdir (vec3))
                        (c0 (point))
                        (c1 (point)))
                    (declare (dynamic-extent best-dir
                                             b0 b1
                                             cdir c0 c1))
                    (loop for j from 0 to 2
                          do (p<- c0 (if (= j 0) s2 s1))
                             (p<- c1 (if (< j 2) s3 s2))
                             (let ((r (line c0 c1 cdir))
                                   (d* (vsqrdistance cdir tootbird)))
                               (declare (type (unsigned-byte 4) r))
                               (when (< d* d)
                                 (setf d d*
                                       best-dim r)
                                 (v<- best-dir cdir)
                                 (p<- b0 c0)
                                 (when (> r 1) (p<- b1 c1)))))
                    ;; should always find a solution
                    (assert (/= d MOST-POSITIVE-SINGLE-FLOAT))
                    ;; copy results to output
                    (v<- dir best-dir)
                    (p<- s1 b0)
                    (when (> best-dim 1) (p<- s2 b1))
                    (setf dim best-dim))))
           (cond
             (flat
              (flat))
             (T
              ;; otherwise pick axis-aligned plane with largest projection
              (loop for i below 3
                    for u of-type single-float = (+ (projected-cross s1 s2 i)
                                                    (projected-cross s2 s3 i)
                                                    (projected-cross s3 s1 i))
                    when (> (abs u) (abs umax))
                      do (setf umax u
                               j i))
              ;; and project simplex and point onto that plane, and
              ;; find the support
              (flet ((projected-cross (a b x)
                       (macrolet ((c (x y)
                                    `(- (* (,x a) (,y b))
                                        (* (,y a) (,x b)))))
                         (ecase x
                           (0 (c vy vz))
                           (1 (- (c vx vz)))
                           (2 (c vx vy))))))
                (let* ((c3 (+ (projected-cross p0 s1 j)
                              (projected-cross s1 s2 j)
                              (projected-cross s2 p0 j)))
                       (c1 (+ (projected-cross p0 s2 j)
                              (projected-cross s2 s3 j)
                              (projected-cross s3 p0 j)))
                       (c2 (+ (projected-cross p0 s3 j)
                              (projected-cross s3 s1 j)
                              (projected-cross s1 p0 j)))
                       (in (or (and (plusp umax)
                                    (plusp c1) (plusp c2) (plusp c3))
                               (and (minusp umax)
                                    (minusp c1) (minusp c2) (minusp c3)))))
                  (declare (dynamic-extent c1 c2 c3))

                  (cond
                    ;; support is entire triangle
                    (in
                     (!v* dir s1 (/ c1 umax))
                     (!v+* dir dir s2 (/ c2 umax))
                     (!v+* dir dir s3 (/ c3 umax))
                     3)
                    ;; otherwise, try edges and pick best
                    (T
                     ;; todo: only try edges with sign mismatch
                     (flat))))))))))))
  (let ((dist (vsqrdistance dir tootbird)))
    (cond
      ;; if tootbird is on simplex, we are done, and simplex is
      ;; face/edge/point of difference (locally) closest to origin
      ((and
        ;; coarse test against fixed distance
        (< dist 1e-4)
        ;; then test against epsilon based on size of simplex
        (< dist (* 2 short-float-epsilon
                   (ecase dim
                     (1 (vinorm s1))
                     (2 (+ (vinorm s1) (vinorm s2)))
                     (3 (+ (vinorm s1) (vinorm s2) (vinorm s3)))))))
       (values dim 1 best))
      (T
       ;; otherwise update dir as direction from closest point to
       ;; tootbird
       (!v- dir tootbird dir)
       (nvunit dir)
       ;; return new simplex size and that we aren't done
       (values dim 0 best)))))

(defun %sttb (a b hit)
  (let ((s0 (point))
        (s1 (point))
        (s2 (point))
        (s3 (point))
        (dir (vec3))
        (dim 0)
        (ret 0)
        (tootbird (vec3))
        (best-distance MOST-POSITIVE-SINGLE-FLOAT))
    (declare (dynamic-extent s0 s1 s2 s3 dir tootbird)
             (type (unsigned-byte 2) dim)
             (type (signed-byte 2) ret)
             (optimize speed))
    ;; website suggests "direction from center of Minkowski difference
    ;; to origin" as first direction, so approximate that with
    ;; difference of centers
    (trial:global-location a dir)
    (trial:global-location b s0)
    (!v- dir dir s0)
    (nvunit dir)
    (when (v~= dir 0.0)
      ;; no initial guess for best direction, just pick something
      (v<- dir +vx3+))
    (loop for i below STTB-ITERATIONS
          do (search-point s0 dir a b)
             (setf (values dim ret best-distance)
                   (update-simplex dim s0 s1 s2 s3
                                   dir tootbird
                                   best-distance))
          while (zerop ret))

    (let ((b-point (vec3))
          (a-point (vec3))
          (p (vec3)))
      (declare (dynamic-extent a-point b-point p))
      ;; calculate hit location from tootbird and simplex
      (ecase dim
        (0
         (v<- a-point (point-a s0))
         (v<- b-point (point-b s0)))
        (1
         (v<- a-point (point-a s1))
         (v<- b-point (point-b s1)))
        (2
         (let* ((ds (v- s2 s1))
                (dt (v- tootbird s1))
                (l (vsqrlength ds)))
           (declare (dynamic-extent ds dt))
           (cond
             ((< l 0.000001)
              ;; simplex is small, use the midpoint of smaller edge
              ;; on original shapes as hit location, and tootbird as
              ;; normal
              (let ((p1 (+ (vsqrdistance (point-a s1) (point-a s2))))
                    (p2 (+ (vsqrdistance (point-b s1) (point-b s2)))))
                (cond
                  ((<= p1 p2)
                   (!v+ a-point (point-a s1) (point-a s2))
                   (nv* a-point 0.5f0)
                   (!v+ b-point a-point tootbird))
                  (T
                   (!v+ b-point (point-b s1) (point-b s2))
                   (nv* b-point 0.5f0)
                   (!v- a-point b-point tootbird)))))
             (T
              (setf l (/ (v. ds dt) l))
              (!v- ds (point-a s2) (point-a s1))
              (!v+* a-point (point-a s1) ds l)
              (!v- ds (point-b s2) (point-b s1))
              (!v+* b-point (point-b s1) ds l)))))
        (3
         (cond
           ((barycentric s1 s2 s3
                         ;; not sure if this should be looking for
                         ;; point close to tootbird or origin?
                         #++ tootbird (vec3)
                         p)
            (nv+* a-point (point-a s1) (vx p))
            (nv+* a-point (point-a s2) (vy p))
            (nv+* a-point (point-a s3) (vz p))
            (nv+* b-point (point-b s1) (vx p))
            (nv+* b-point (point-b s2) (vy p))
            (nv+* b-point (point-b s3) (vz p)))
           (T
            ;; barycentric failed (not sure if this still happens
            ;; with it using doubles?). Use tootbird for normal,
            ;; then pick whichever shape had smaller simplex, and
            ;; use middle of that simplex as hit location. Using
            ;; perimeter rather than area since it is pretty common
            ;; for one of the shapes to be supported by a single
            ;; edge, so 0 area doesn't indicate smaller region.
            (let ((p1 (+ (vdistance (point-a s1) (point-a s2))
                         (vdistance (point-a s2) (point-a s3))
                         (vdistance (point-a s3) (point-a s1))))
                  (p2 (+ (vdistance (point-b s1) (point-b s2))
                         (vdistance (point-b s2) (point-b s3))
                         (vdistance (point-b s3) (point-b s1)))))
              (v<- (trial:hit-normal hit) tootbird)
              (cond
                ((<= p1 p2)
                 (!v+ a-point (point-a s1) (point-a s2))
                 (nv+ a-point (point-a s3))
                 (nv* a-point 0.33333334f0)
                 (!v+ b-point a-point tootbird))
                (T
                 (!v+ b-point (point-b s1) (point-b s2))
                 (nv+ b-point (point-b s3))
                 (nv* b-point 0.33333333f0)
                 (!v- a-point b-point tootbird))))))))
      (v<- (trial:hit-location hit) a-point)
      (v<- (trial:hit-normal hit) b-point)
      (nv- (trial:hit-normal hit) a-point)
      (setf (trial:hit-depth hit) (vlength (trial:hit-normal hit)))
      (if (= 0.0 (trial:hit-depth hit))
          (v<- (trial:hit-normal hit) +vy3+)
          (nv/ (trial:hit-normal hit) (trial:hit-depth hit))))
    (plusp ret)))

(defun detect-hits (a b hits start end)
  (declare (type trial:primitive a b))
  (declare (type (unsigned-byte 32) start end))
  (declare (type simple-vector hits))
  (declare (optimize speed))

  (when (<= end start)
    (return-from detect-hits start))
  (let ((hit (aref hits start)))
    (prog1
        (cond ((%sttb a b hit)
               (trial:finish-hit hit a b)
               (1+ start))
              (T
               start)))))

#++
(trial:define-hit-detector (trial:primitive trial:primitive)
  (setf trial:start (detect-hits a b trial:hits trial:start trial:end)))
