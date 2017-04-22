(in-package #:org.shirakumo.fraf.trial)

(defvar *dialogues* (make-hash-table))

(defun dialogue-action (action)
  (format NIL "> ~{~#[~;~@(~a~)..~;~@(~a~) ~(~a~)s~:;~@(~a~) ~(~a~)s~@[ at ~#[~;~(~a~)~;~(~a~) and ~(~a~)~:;~(~a~)~@{~#[~;, and ~(~a~)~:;, ~(~a~)~]~}~]~]~]~}." action))

(defun dialogue-next (dialogue &optional choice))

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
                          (loop for choice in (pop dialogue)
                                collect (let ((choice-dialog (list :text (car choice))))
                                          (setf (cddr choice-dialog)
                                                (cdr choice))
                                          (when (getf choice-dialog :action)
                                            (setf (getf choice-dialog :action)
                                                  (dialogue-action (getf choice-dialog :action))))
                                          choice-dialog)))))))
               (symbol
                (append-to-next (list :actor part :text (pop dialogue)))
                (next-part))))
        finally (return dialog)))
