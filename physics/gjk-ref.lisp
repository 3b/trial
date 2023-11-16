(defun rat-line-plane (rs rd pn p0)
  ;; line p=rs+t*rd, plane (p-p₀)*n=0
  (flet ((rat (x) (rational x)))
    (let* ((l0 (map 'vector #'rat rs))
           (l (map 'vector #'rat rd))
           (n (map 'vector #'rat pn))
                                        ;(dl (print (reduce '+ (map 'vector (alexandria:rcurry 'expt 2) n))))
           (p0 (map 'vector #'rat p0))
           (l⋅n (reduce '+ (map 'vector '* l n)))
           (p0-l0 (map 'vector '- p0 l0))
           (p0-l0⋅n (reduce '+ (map 'vector '* p0-l0 n)))
           (d (unless (zerop l⋅n)
                (/ p0-l0⋅n l⋅n)))
           (r (when d
                (map 'vector '+ l0 (map 'vector (alexandria:curry '* d) l)))))
      (values (map 'vector 'float r) r))))


(rat-line-plane #(3.1813586 2.4607022 3.5005364)
                #(-0.5803402 -0.6600952 -0.4769482)
                #(0 0 1)
                #(0 0 1))
#(0.13876018 -1.0000345 1.0)
#(4657103149925/33562245988352 -33563404127421/33562245988352 1)
#(-2.2947965 -3.76803 -1.0)
#(-77018519921819/33562245988352 -126463546739901/33562245988352 -1)

#(0.13879052 -1.0 1.0000249)
#(6446829253759/46450071306240 -1 46451229445309/46450071306240)
#(0.13879089 -1.0 1.000025)
#(70489005004919/507879164810415 -1 27878184449325/27877485553352)
#(0.13879052 -1.0 1.0000249)
#(6446829253759/46450071306240 -1 46451229445309/46450071306240)
(cos (/ pi 16))
0.9807852804032304d0
0.9987954562051724d0
0.049126849769467254d0
0.049067674327418015d0

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

(rat-point-line #(2.971071 2.9997387 3.965216)
                #(-0.528905 -0.6373113 -0.56044066)
                #(0 0 0))
0.9982341
0.9964713
4934286041137922652266783723/4951759524220230983363854336
#(-0.046570145 -0.6364093 0.7676495)
