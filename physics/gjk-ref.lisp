;;; brute-force ray / convex-mesh intersection test, just for
;;; testing.
(in-package #:org.shirakumo.fraf.trial.gjk)

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


;; todo: finish full rational implementation if possible?

(defun rat-line-plane (rs rd pn p0)
  ;; line p=rs+t*rd, plane (p-p₀)*n=0
  (flet ((rat (x) (rational x)))
    (let* ((l0 (map 'vector #'rat rs))
           (l (map 'vector #'rat rd))
           (n (map 'vector #'rat pn))
           ;;(dl (print (reduce '+ (map 'vector (alexandria:rcurry 'expt 2) n))))
           (p0 (map 'vector #'rat p0))
           (l⋅n (reduce '+ (map 'vector '* l n)))
           (p0-l0 (map 'vector '- p0 l0))
           (p0-l0⋅n (reduce '+ (map 'vector '* p0-l0 n)))
           (d (unless (zerop l⋅n)
                (/ p0-l0⋅n l⋅n)))
           (r (when d
                (map 'vector '+ l0 (map 'vector (alexandria:curry '* d) l)))))
      (values (map 'vector 'float r) r))))


(defun rat-point-line (rs rd p0)
  ;; line p=rs+t*rd, point p0
  (flet ((rat (x) (rational x)))
    (let* ((b (map 'vector #'rat rs))
           (m (map 'vector #'rat rd))
           (p (map 'vector #'rat p0))
           (p-b (map 'vector '- p b))
           (m⋅p-b (reduce '+ (map 'vector '* m p-b)))
           (m⋅m (reduce '+ (map 'vector '* m m)))
           (t0 (unless (zerop m⋅m)
                 (/ m⋅p-b m⋅m)))
           (p1 (when t0
                 (map 'vector '+ b (map 'vector (alexandria:curry '* t0) m))))
           (d² (when p1
                 (reduce '+ (map 'vector (alexandria:rcurry 'expt 2)
                                 (map 'vector '- p1 p))))))
      (values (sqrt d²) (float d²) d²
              (map 'vector 'float p1)))))
