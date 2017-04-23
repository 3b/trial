(in-package #:org.shirakumo.fraf.trial)

(defvar *dialogues* (make-hash-table))

(defmethod dialogue ((name symbol))
  (gethash name *dialogues*))

(defmethod (setf dialogue) (value (name symbol))
  (setf (gethash name *dialogues*) value))

(defun remove-dialogue (name)
  (remhash name *dialogues*))

(defun dialogue-action (action)
  (format NIL "> ~{~#[~;~@(~a~)..~;~@(~a~) ~(~a~)s~:;~@(~a~) ~(~a~)s~@[ at ~#[~;~(~a~)~;~(~a~) and ~(~a~)~:;~(~a~)~@{~#[~;, and ~(~a~)~:;, ~(~a~)~]~}~]~]~]~}." action))

(defun dialogue-find (dialogue tag)
  (loop for forwards = (getf dialogue :next) then (When forwards (getf forwards :next))
        for backwards = (getf dialogue :prev) then (when backwards (getf backwards :prev))
        while (or forwards backwards)
        until (or (find tag (getf forwards :tags)) (find tag (getf backwards :tags)))
        finally (return (cond
                          ((find tag (getf forwards :tags)) forwards)
                          ((find tag (getf backwards :tags)) backwards)
                          (T NIL)))))

(defun dialogue-next (dialogue &optional choice)
  (if choice
      (let ((choice (etypecase choice
                      (integer (nth choice (getf dialogue :choice)))
                      (list choice))))
        (if (getf choice :go)
            (dialogue-find dialogue (getf choice :go))
            (dialogue (getf choice :dialogue))))
      (let ((next (getf dialogue :next)))
        (setf (getf dialogue :next) NIL
              (getf next :prev) (copy-list dialogue))
        next)))

(defmacro define-dialogue (name &rest dialogue)
  (loop with dialog = (list :name name)
        with current = dialog
        with next = NIL
        for part = (car dialogue)
        while part
        do (flet ((append-to-next (appendable)
                    (if next
                        (setf (cdr (last next)) appendable)
                        (setf next appendable)))
                  (append-to-property (appendable property)
                    (if next
                        (setf (getf next property) (append (getf next property)
                                                           (list appendable)))
                        (setf next (list property (list appendable)))))
                  (next-part ()
                    (setf (cdr (last current)) (list :next next)
                          current next
                          next NIL)))
             (pop dialogue)
             (etypecase part
               (keyword
                (case (alexandria:ensure-symbol part :trial)
                  (tag
                   (append-to-property (pop dialogue) :tags))
                  (action
                   (append-to-property (dialogue-action (pop dialogue)) :actions))
                  (choice
                   (append-to-next
                    (list :choice
                          (loop for (text . rest) in (pop dialogue)
                                for choice = (cons :text (cons text rest))
                                for action = (getf choice :action)
                                when action do (setf (getf choice :action) (dialogue-action action))
                                collect choice))))))
               (symbol
                (append-to-next (list :actor part :text (pop dialogue)))
                (next-part))))
        finally (return `(progn
                           (setf (dialogue ',name) ',dialog)
                           ',name))))
