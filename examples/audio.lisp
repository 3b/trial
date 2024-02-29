(in-package #:org.shirakumo.fraf.trial.examples)

(defmethod trial-harmony:server-initargs append ((main main))
  (list :mixers '((:music mixed:basic-mixer)
                  (:effect mixed:plane-mixer))
        :effects '((mixed:biquad-filter :filter :lowpass :name :lowpass)
                   (mixed:speed-change :name :speed))))

(define-example audio
  :title "Audio"
  (setf (mixed:min-distance harmony:*server*) 10)
  (setf (mixed:max-distance harmony:*server*) (min (width *context*) (height *context*)))
  (preload (assets:// :cave-ambience) scene)
  (preload (assets:// :step-rocks) scene)
  (enter (make-instance 'render-pass) scene))

(defmethod setup-ui ((scene audio-scene) panel)
  (let* ((layout (make-instance 'alloy:grid-layout :col-sizes '(T 120 200) :row-sizes '(30)))
         (focus (make-instance 'alloy:vertical-focus-list))
         (row 0))
    (macrolet ((row (label repr input &rest args)
                 `(prog2 (alloy:enter ,label layout :row row :col 1)
                      (alloy:represent ,repr ,input ,@args :focus-parent focus :layout-parent layout)
                    (incf row))))
      (row "Master Volume" (mixed:volume :master) 'alloy:ranged-slider :range '(0.0 . 1.0))
      (row "Music Volume" (mixed:volume :music) 'alloy:ranged-slider :range '(0.0 . 1.0))
      (row "Effect Volume" (mixed:volume :effect) 'alloy:ranged-slider :range '(0.0 . 1.0))
      (row "Lowpass Frequency" (mixed:frequency (harmony:segment :lowpass T)) 'alloy:ranged-slider :range '(100.0 . 10000.0))
      (row "Speed Factor" (mixed:speed-factor (harmony:segment :speed T)) 'alloy:ranged-slider :range '(0.5 . 2.0))
      (let* ((state NIL)
             (switch (row "Music Track" state 'alloy:switch)))
        (alloy:on alloy:value (value switch)
          (setf (harmony:repeat (assets:// :cave-ambience)) T)
          (if value
              (harmony:play (assets:// :cave-ambience) :mixer :music)
              (harmony:stop (assets:// :cave-ambience))))))
    (alloy:finish-structure panel layout focus)))

(define-handler (audio-scene mouse-press :before) (pos)
  (harmony:play (assets:// :step-rocks)
                :reset T
                :location (v- pos (v* (size *context*) 0.5))))

(define-handler (audio-scene resize :before) (width height)
  (setf (mixed:max-distance harmony:*server*) (min width height)))
